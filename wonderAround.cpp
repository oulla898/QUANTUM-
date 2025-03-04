#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <driver/adc.h>

// --- Configuration ---

// --- VL53L0X ---
// I2C Addresses (after initialization)
#define VL53L0X_ADDRESS_1 0x30
#define VL53L0X_ADDRESS_2 0x31
#define VL53L0X_ADDRESS_3 0x32
#define VL53L0X_ADDRESS_4 0x33

// XSHUT Pins
#define XSHUT1_PIN 38
#define XSHUT2_PIN 39
#define XSHUT3_PIN 21
#define XSHUT4_PIN 42

// --- Motor Driver Pins ---
#define PWMA_PIN 4
#define AIN2_PIN 5
#define AIN1_PIN 6
#define STBY_PIN 7
#define BIN1_PIN 8
#define BIN2_PIN 9
#define PWMB_PIN 10
#define ADC_PIN 3

// --- Encoder Pins ---
#define E1A_PIN 15
#define E1B_PIN 16
#define E2A_PIN 17
#define E2B_PIN 18

// --- Motor Control ---
#define MOTOR_A 1
#define MOTOR_B 2
#define FORWARD 1
#define REVERSE -1
#define BRAKE 0
#define COAST 0

// --- Battery Monitoring ---
#define VOLTAGE_DIVIDER_RATIO (11.0)  // 10k / 1k divider
#define ADC_REFERENCE_VOLTAGE (3.3) // ESP32 ADC reference

// --- PID Control ---
#define CONTROL_LOOP_INTERVAL_US (10000) // 10ms control loop interval
#define ENCODER_COUNTS_PER_REV (52) // 13 PPR * 4 state changes per cycle (quadrature)

// --- PID Gains (Separate for Each Motor) ---
// Motor A
float KPA = 6;
float KIA = 1.8;
float KDA = 0;
// Motor B
float KPB = 6;
float KIB = 1.8;
float KDB = 0;

// --- Create Sensor Objects ---
Adafruit_VL53L0X vl53_1 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl53_2 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl53_3 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl53_4 = Adafruit_VL53L0X();

// --- Global Variables ---
volatile long encoderA_count = 0;
volatile long encoderB_count = 0;
float desired_speed_A = 10; // Initialize to 10
float desired_speed_B = 10; // Initialize to 10
float current_speed_A = 0;
float current_speed_B = 0;
float pid_output_A = 0;
float pid_output_B = 0;
float error_sum_A = 0;
float error_sum_B = 0;
float last_error_A = 0;
float last_error_B = 0;
bool motors_enabled = false;

// --- Obstacle Avoidance ---
int obstacle_threshold = 200; // Threshold for obstacle detection in mm
bool obstacle_detected = false;
unsigned long obstacle_avoidance_start_time = 0;
#define OBSTACLE_AVOIDANCE_DURATION 1000 // Duration of obstacle avoidance maneuver in ms
enum AvoidanceState {
  AVOID_NONE,
  AVOID_TURN_LEFT,
  AVOID_TURN_RIGHT,
  AVOID_DRIVE_FORWARD
};
AvoidanceState avoidance_state = AVOID_NONE;

// --- Lookup Table for Quadrature Decoding ---
const int8_t encoder_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// --- Function Prototypes ---
void IRAM_ATTR encoderA_isr();
void IRAM_ATTR encoderB_isr();
void control_loop(void* arg);
void setMotorSpeed(int motor, float speed);
void setMotorDirection(int motor, int direction);
void enableMotors();
void disableMotors();
float readBatteryVoltage();
float calculateSpeed(long encoder_count, long interval_us);

void initializeVL53L0X(Adafruit_VL53L0X &sensor, int xshutPin, uint8_t newAddress);
void printVL53L0XResult(VL53L0X_RangingMeasurementData_t measure);
void avoidObstacle();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for serial monitor to open

    // Initialize I2C
  Wire.begin(40, 41); // SDA, SCL

  // --- VL53L0X Initialization ---

  // Configure XSHUT pins as outputs
  pinMode(XSHUT1_PIN, OUTPUT);
  pinMode(XSHUT2_PIN, OUTPUT);
  pinMode(XSHUT3_PIN, OUTPUT);
  pinMode(XSHUT4_PIN, OUTPUT);

  // Disable all sensors initially
  digitalWrite(XSHUT1_PIN, LOW);
  digitalWrite(XSHUT2_PIN, LOW);
  digitalWrite(XSHUT3_PIN, LOW);
  digitalWrite(XSHUT4_PIN, LOW);
  delay(50); // Short delay

  // Initialize each sensor one by one
  initializeVL53L0X(vl53_1, XSHUT1_PIN, VL53L0X_ADDRESS_1);
  initializeVL53L0X(vl53_2, XSHUT2_PIN, VL53L0X_ADDRESS_2);
  initializeVL53L0X(vl53_3, XSHUT3_PIN, VL53L0X_ADDRESS_3);
  initializeVL53L0X(vl53_4, XSHUT4_PIN, VL53L0X_ADDRESS_4);

  // Configure motor driver pins
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);

  // Configure encoder pins
  pinMode(E1A_PIN, INPUT_PULLUP);
  pinMode(E1B_PIN, INPUT_PULLUP);
  pinMode(E2A_PIN, INPUT_PULLUP);
  pinMode(E2B_PIN, INPUT_PULLUP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(E1A_PIN), encoderA_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E1B_PIN), encoderA_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E2A_PIN), encoderB_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(E2B_PIN), encoderB_isr, CHANGE);

  // Add 0.1uF capacitors between each encoder pin (E1A, E1B, E2A, E2B) and ground

  // Configure ADC for battery monitoring
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // Assuming ADC1_CHANNEL_0 corresponds to GPIO3

  // Initialize motors to enabled
  enableMotors();
  setMotorSpeed(MOTOR_A, 10);
  setMotorSpeed(MOTOR_B, 10);

  // Create and start the control loop timer
  esp_timer_create_args_t timer_args = {
    .callback = &control_loop,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "control_loop_timer"
  };
  esp_timer_handle_t control_loop_timer;
  esp_timer_create(&timer_args, &control_loop_timer);
  esp_timer_start_periodic(control_loop_timer, CONTROL_LOOP_INTERVAL_US);
}

// --- Main Loop ---
//=================================================================================================================
void loop() {
  avoidObstacle();

  // --- Read Battery Voltage ---
  float battery_voltage = readBatteryVoltage();

  // --- Print Information to Serial ---
  Serial.print("Battery: ");
  Serial.print(battery_voltage);
  Serial.print("V, ");

  Serial.print("Speed A: ");
  Serial.print(current_speed_A);
  Serial.print(", ");

  Serial.print("Desired A: ");
  Serial.print(desired_speed_A);
  Serial.print(", ");

  Serial.print("Encoder A: ");
  Serial.print(encoderA_count);
  Serial.print(", ");

  Serial.print("Speed B: ");
  Serial.print(current_speed_B);
  Serial.print(", ");

  Serial.print("Desired B: ");
  Serial.print(desired_speed_B);
  Serial.print(", ");

  Serial.print("Encoder B: ");
  Serial.println(encoderB_count);

  // --- Serial Command Interface ---
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    if (command.startsWith("SA")) { // Set speed for Motor A
      float speed = command.substring(2).toFloat();
      setMotorSpeed(MOTOR_A, speed);
      Serial.print("Setting Motor A Speed: ");
      Serial.println(speed);
    } else if (command.startsWith("SB")) { // Set speed for Motor B
      float speed = command.substring(2).toFloat();
      setMotorSpeed(MOTOR_B, speed);
      Serial.print("Setting Motor B Speed: ");
      Serial.println(speed);
    } else if (command == "E") { // Enable motors
      enableMotors();
      Serial.println("Motors Enabled");
    } else if (command == "D") { // Disable motors
      disableMotors();
      Serial.println("Motors Disabled");
    } else if (command == "STOP") { // Stop both motors
      setMotorSpeed(MOTOR_A, 0);
      setMotorSpeed(MOTOR_B, 0);
      Serial.println("Motors Stopped");
    } else if (command.startsWith("KPA")) { // Set KP for Motor A
      KPA = command.substring(3).toFloat();
      Serial.print("KPA set to: ");
      Serial.println(KPA);
    } else if (command.startsWith("KIA")) { // Set KI for Motor A
      KIA = command.substring(3).toFloat();
      Serial.print("KIA set to: ");
      Serial.println(KIA);
    } else if (command.startsWith("KDA")) { // Set KD for Motor A
      KDA = command.substring(3).toFloat();
      Serial.print("KDA set to: ");
      Serial.println(KDA);
    } else if (command.startsWith("KPB")) { // Set KP for Motor B
      KPB = command.substring(3).toFloat();
      Serial.print("KPB set to: ");
      Serial.println(KPB);
    } else if (command.startsWith("KIB")) { // Set KI for Motor B
      KIB = command.substring(3).toFloat();
      Serial.print("KIB set to: ");
      Serial.println(KIB);
    } else if (command.startsWith("KDB")) { // Set KD for Motor B
      KDB = command.substring(3).toFloat();
      Serial.print("KDB set to: ");
      Serial.println(KDB);
    } else if (command == "HELP") { // Display help message
      Serial.println("--- Available Commands ---");
      Serial.println("SA[value] - Set speed for Motor A");
      Serial.println("SB[value] - Set speed for Motor B");
      Serial.println("E          - Enable motors");
      Serial.println("D          - Disable motors");
      Serial.println("STOP       - Stop both motors");
      Serial.println("KPA[value] - Set KP for Motor A");
      Serial.println("KIA[value] - Set KI for Motor A");
      Serial.println("KDA[value] - Set KD for Motor A");
      Serial.println("KPB[value] - Set KP for Motor B");
      Serial.println("KIB[value] - Set KI for Motor B");
      Serial.println("KDB[value] - Set KD for Motor B");
      Serial.println("HELP       - Display this help message");
    } else {
      Serial.print("Unknown command: ");
      Serial.println(command);
    }
  }

  delay(50); // Adjust delay as needed (keep it relatively short for responsiveness)
}

// --- Interrupt Service Routines (ISRs) for Encoders ---
void IRAM_ATTR encoderA_isr() {
  static uint8_t encoderA_state = 0;
  encoderA_state = (encoderA_state << 2) | (digitalRead(E1A_PIN) << 1) | digitalRead(E1B_PIN);
  encoderA_count += encoder_table[encoderA_state & 0x0F];
}

void IRAM_ATTR encoderB_isr() {
  static uint8_t encoderB_state = 0;
  encoderB_state = (encoderB_state << 2) | (digitalRead(E2A_PIN) << 1) | digitalRead(E2B_PIN);
  encoderB_count += encoder_table[encoderB_state & 0x0F];
}

// --- Control Loop (Timer-based) ---
void control_loop(void* arg) {
  static long last_encoderA_count = 0;
  static long last_encoderB_count = 0;
  static unsigned long last_time_us = 0;

  unsigned long current_time_us = esp_timer_get_time();
  unsigned long elapsed_time_us = current_time_us - last_time_us;
  last_time_us = current_time_us;

  // Calculate speeds (counts per interval) in the correct direction
  current_speed_A = last_encoderA_count - encoderA_count; // Reversed for Motor A
  current_speed_B = encoderB_count - last_encoderB_count;
  last_encoderA_count = encoderA_count;
  last_encoderB_count = encoderB_count;

  // --- PID Control for Motor A ---
  float error_A = desired_speed_A - current_speed_A;
  error_sum_A += error_A;
  error_sum_A = constrain(error_sum_A, -100, 100); // Prevent integral windup
  float error_diff_A = error_A - last_error_A;
  last_error_A = error_A;

  // Use separate PID gains for Motor A
  pid_output_A = (KPA * error_A) + (KIA * error_sum_A) + (KDA * error_diff_A);
  pid_output_A = constrain(pid_output_A, -255, 255); // Limit output

  // --- PID Control for Motor B ---
  float error_B = desired_speed_B - current_speed_B;
  error_sum_B += error_B;
  error_sum_B = constrain(error_sum_B, -100, 100); // Prevent integral windup
  float error_diff_B = error_B - last_error_B;
  last_error_B = error_B;

  // Use separate PID gains for Motor B
  pid_output_B = (KPB * error_B) + (KIB * error_sum_B) + (KDB * error_diff_B);
  pid_output_B = constrain(pid_output_B, -255, 255); // Limit output

  // Update motor outputs based on PID
  if (motors_enabled) {
    if (pid_output_A >= 0) {
      setMotorDirection(MOTOR_A, FORWARD);
      analogWrite(PWMA_PIN, (int)pid_output_A);
    } else {
      setMotorDirection(MOTOR_A, REVERSE);
      analogWrite(PWMA_PIN, (int)abs(pid_output_A));
    }

    if (pid_output_B >= 0) {
      setMotorDirection(MOTOR_B, FORWARD);
      analogWrite(PWMB_PIN, (int)pid_output_B);
    } else {
      setMotorDirection(MOTOR_B, REVERSE);
      analogWrite(PWMB_PIN, (int)abs(pid_output_B));
    }
  } else {
    // Stop motors if not enabled
    analogWrite(PWMA_PIN, 0);
    analogWrite(PWMB_PIN, 0);
  }
}

// --- Helper Functions ---
void setMotorSpeed(int motor, float speed) {
  if (motor == MOTOR_A) {
    desired_speed_A = speed;
  } else if (motor == MOTOR_B) {
    desired_speed_B = speed;
  }
}

void setMotorDirection(int motor, int direction) {
  if (motor == MOTOR_A) {
    if (direction == FORWARD) {
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, LOW);
    } else if (direction == REVERSE) {
      digitalWrite(AIN1_PIN, LOW);
      digitalWrite(AIN2_PIN, HIGH);
    } else if (direction == BRAKE) {
      digitalWrite(AIN1_PIN, HIGH);
      digitalWrite(AIN2_PIN, HIGH);
    } else { // COAST
      digitalWrite(AIN1_PIN, LOW);
      digitalWrite(AIN2_PIN, LOW);
    }
  } else if (motor == MOTOR_B) {
    if (direction == FORWARD) {
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, LOW);
    } else if (direction == REVERSE) {
      digitalWrite(BIN1_PIN, LOW);
      digitalWrite(BIN2_PIN, HIGH);
    } else if (direction == BRAKE) {
      digitalWrite(BIN1_PIN, HIGH);
      digitalWrite(BIN2_PIN, HIGH);
    } else { // COAST
      digitalWrite(BIN1_PIN, LOW);
      digitalWrite(BIN2_PIN, LOW);
    }
  }
}

void enableMotors() {
  digitalWrite(STBY_PIN, HIGH);
  motors_enabled = true;
}

void disableMotors() {
  digitalWrite(STBY_PIN, LOW);
  motors_enabled = false;
}

float readBatteryVoltage() {
  int adc_value = adc1_get_raw(ADC1_CHANNEL_0); // Read ADC value
  float voltage = (float)adc_value * ADC_REFERENCE_VOLTAGE / 4095.0; // Convert to voltage (12-bit ADC)
  return voltage * VOLTAGE_DIVIDER_RATIO; // Scale by the voltage divider ratio
}

float calculateSpeed(long encoder_count, long interval_us) {
  // Calculate speed based on encoder counts and time interval
  float speed = (float)encoder_count / (float)ENCODER_COUNTS_PER_REV * (1000000.0 / (float)interval_us); // Example: counts per second
  return speed;
}

void initializeVL53L0X(Adafruit_VL53L0X &sensor, int xshutPin, uint8_t newAddress) {
  // Bring sensor out of reset
  digitalWrite(xshutPin, HIGH);
  delay(10); // Short delay

  // Initialize sensor (with retries)
  while (!sensor.begin(0x29, &Wire)) { // Default address is 0x29
    Serial.print("Failed to begin VL53L0X sensor with XSHUT pin ");
    Serial.println(xshutPin);
    delay(1000);
  }

  // Change sensor address
  sensor.setAddress(newAddress);
  Serial.print("VL53L0X sensor with XSHUT pin ");
  Serial.print(xshutPin);
  Serial.print(" initialized with address 0x");
  Serial.println(newAddress, HEX);
}

void printVL53L0XResult(VL53L0X_RangingMeasurementData_t measure) {
  if (measure.RangeStatus != 4) {  // 4 means out of range
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println("Out of range");
  }
}

void avoidObstacle() {
  VL53L0X_RangingMeasurementData_t measure;
  int dist1, dist2, dist3, dist4;

  // Read sensor data
  vl53_1.rangingTest(&measure, false);
  dist1 = measure.RangeMilliMeter;
  vl53_2.rangingTest(&measure, false);
  dist2 = measure.RangeMilliMeter;
  vl53_3.rangingTest(&measure, false);
  dist3 = measure.RangeMilliMeter;
  vl53_4.rangingTest(&measure, false);
  dist4 = measure.RangeMilliMeter;

  // Print sensor readings for debugging
  Serial.print("VL53L0X 1: ");
  Serial.print(dist1);
  Serial.print(", ");
  Serial.print("VL53L0X 2: ");
  Serial.print(dist2);
  Serial.print(", ");
  Serial.print("VL53L0X 3: ");
  Serial.print(dist3);
  Serial.print(", ");
  Serial.print("VL53L0X 4: ");
  Serial.println(dist4);

  // Obstacle detection logic
  if (dist3 < obstacle_threshold || dist4 < obstacle_threshold) {
    if (!obstacle_detected) {
      obstacle_detected = true;
      obstacle_avoidance_start_time = millis();

      // Determine initial avoidance direction
      if (dist3 < dist4) {
        avoidance_state = AVOID_TURN_LEFT;
      } else {
        avoidance_state = AVOID_TURN_RIGHT;
      }
    }
  } else {
    obstacle_detected = false;
  }

  // Obstacle avoidance maneuver
  if (obstacle_detected) {
    unsigned long current_time = millis();
    if (current_time - obstacle_avoidance_start_time < OBSTACLE_AVOIDANCE_DURATION) {
      if (avoidance_state == AVOID_TURN_LEFT) {
        setMotorSpeed(MOTOR_A, -10);
        setMotorSpeed(MOTOR_B, 10);
        Serial.println("Turning Left");
      } else if (avoidance_state == AVOID_TURN_RIGHT) {
        setMotorSpeed(MOTOR_A, 10);
        setMotorSpeed(MOTOR_B, -10);
        Serial.println("Turning Right");
      } else if (avoidance_state == AVOID_DRIVE_FORWARD) {
        setMotorSpeed(MOTOR_A, 10);
        setMotorSpeed(MOTOR_B, 10);
        Serial.println("Driving Forward");
      }
    } else {
       // Check if path is clear after turning
        if(dist1 > obstacle_threshold && dist2 > obstacle_threshold){
          avoidance_state = AVOID_DRIVE_FORWARD;
        }
    }
  } else {
    avoidance_state = AVOID_NONE;
    setMotorSpeed(MOTOR_A, 10);
    setMotorSpeed(MOTOR_B, 10);
  }
}
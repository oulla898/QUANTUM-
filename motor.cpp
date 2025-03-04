#include <Arduino.h>
#include <driver/adc.h>
#include <esp_timer.h>

// --- Configuration ---
// --- Motor Driver Pins ---
#define PWMA_PIN 4
#define AIN2_PIN 5
#define AIN1_PIN 6
#define STBY_PIN 7
#define BIN1_PIN 8
#define BIN2_PIN 9
#define PWMB_PIN 10

// --- ADC Pin ---
#define ADC_PIN 36 // Changed to ADC1_CHANNEL_0 which is GPIO36

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
#define VOLTAGE_DIVIDER_RATIO (11.0f)  // 10k / 1k divider, made float for consistency
#define ADC_REFERENCE_VOLTAGE (3.3f) // ESP32 ADC reference, made float for consistency

// --- PID Control ---
#define CONTROL_LOOP_INTERVAL_US (10000) // 10ms control loop interval
#define ENCODER_COUNTS_PER_REV (52)      // 13 PPR * 4 state changes per cycle (quadrature)

// --- PID Gains (Separate for Each Motor) ---
// Motor A
float KPA = 6.0f;
float KIA = 1.8f;
float KDA = 0.0f;
// Motor B
float KPB = 6.0f;
float KIB = 1.8f;
float KDB = 0.0f;

// --- Global Variables ---
volatile long encoderA_count = 0;
volatile long encoderB_count = 0;
float desired_speed_A = 0.0f; // Initialize to 0, can be changed via serial commands
float desired_speed_B = 0.0f; // Initialize to 0, can be changed via serial commands
float current_speed_A = 0.0f;
float current_speed_B = 0.0f;
float pid_output_A = 0.0f;
float pid_output_B = 0.0f;
float error_sum_A = 0.0f;
float error_sum_B = 0.0f;
float last_error_A = 0.0f;
float last_error_B = 0.0f;
bool motors_enabled = false;
esp_timer_handle_t control_loop_timer; // Declare the timer handle globally

// --- Lookup Table for Quadrature Decoding ---
const int8_t encoder_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// --- Function Prototypes ---
void IRAM_ATTR encoderA_isr();
void IRAM_ATTR encoderB_isr();
void control_loop(void *arg);
void setMotorSpeed(int motor, float speed);
void setMotorDirection(int motor, int direction);
void enableMotors();
void disableMotors();
float readBatteryVoltage();

// --- Setup ---
void setup() {
  // Initialize serial communication
  Serial.begin(115200);

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

  // Add 0.1uF capacitors between each encoder pin (E1A, E1B, E2A, E2B) and ground (Hardware improvement - mentioned in code)

  // Configure ADC for battery monitoring
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); 

  // Initialize motors to disabled
  disableMotors();

  // Create and start the control loop timer
  esp_timer_create_args_t timer_args = {
      .callback = &control_loop,
      .arg = NULL,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "control_loop_timer"};
  esp_timer_create(&timer_args, &control_loop_timer);
  esp_timer_start_periodic(control_loop_timer, CONTROL_LOOP_INTERVAL_US);
}

// --- Main Loop ---
void loop() {
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
      desired_speed_A = speed; 
      Serial.print("Setting Motor A Desired Speed: ");
      Serial.println(speed);
    } else if (command.startsWith("SB")) { // Set speed for Motor B
      float speed = command.substring(2).toFloat();
      desired_speed_B = speed;
      Serial.print("Setting Motor B Desired Speed: ");
      Serial.println(speed);
    } else if (command == "E") { // Enable motors
      enableMotors();
      Serial.println("Motors Enabled");
    } else if (command == "D") { // Disable motors
      disableMotors();
      Serial.println("Motors Disabled");
    } else if (command == "STOP") { // Stop both motors
      desired_speed_A = 0;
      desired_speed_B = 0;
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
      Serial.println("SA[value] - Set desired speed for Motor A");
      Serial.println("SB[value] - Set desired speed for Motor B");
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
void control_loop(void *arg) {
  static long last_encoderA_count = 0;
  static long last_encoderB_count = 0;
  static unsigned long last_time_us = 0;

  unsigned long current_time_us = esp_timer_get_time();
  unsigned long elapsed_time_us = current_time_us - last_time_us; 
  last_time_us = current_time_us;

  // Calculate speeds (counts per interval)
  current_speed_A = last_encoderA_count - encoderA_count; // Reversed for Motor A
  current_speed_B = encoderB_count - last_encoderB_count;
  last_encoderA_count = encoderA_count;
  last_encoderB_count = encoderB_count;

  // --- PID Control for Motor A ---
  float error_A = desired_speed_A - current_speed_A;
  error_sum_A += error_A;
  error_sum_A = constrain(error_sum_A, -100.0f, 100.0f); // Prevent integral windup, made float for consistency
  float error_diff_A = error_A - last_error_A;
  last_error_A = error_A;

  // Use separate PID gains for Motor A
  pid_output_A = (KPA * error_A) + (KIA * error_sum_A) + (KDA * error_diff_A);
  pid_output_A = constrain(pid_output_A, -255.0f, 255.0f); // Limit output, made float for consistency

  // --- PID Control for Motor B ---
  float error_B = desired_speed_B - current_speed_B;
  error_sum_B += error_B;
  error_sum_B = constrain(error_sum_B, -100.0f, 100.0f); // Prevent integral windup, made float for consistency
  float error_diff_B = error_B - last_error_B;
  last_error_B = error_B;

  // Use separate PID gains for Motor B
  pid_output_B = (KPB * error_B) + (KIB * error_sum_B) + (KDB * error_diff_B);
  pid_output_B = constrain(pid_output_B, -255.0f, 255.0f); // Limit output, made float for consistency

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
  int adc_value = adc1_get_raw(ADC1_CHANNEL_0); // Read ADC value from ADC1_CHANNEL_0
  float voltage = (float)adc_value * ADC_REFERENCE_VOLTAGE / 4095.0f; // Convert to voltage (12-bit ADC), made float for consistency
  return voltage * VOLTAGE_DIVIDER_RATIO; // Scale by the voltage divider ratio
}
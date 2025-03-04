#include <Wire.h>
#include <Adafruit_VL53L0X.h>

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

// --- MMA7361 ---
#define MMA7361_X_PIN 35
#define MMA7361_Y_PIN 36
#define MMA7361_Z_PIN 37
#define MMA7361_SLEEP_PIN 19

// --- IR Sensors ---
#define IR_SENSOR_1_PIN 2
#define IR_SENSOR_2_PIN 13
#define IR_SENSOR_3_PIN 12
#define IR_SENSOR_4_PIN 14

// --- Create Sensor Objects ---
Adafruit_VL53L0X vl53_1 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl53_2 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl53_3 = Adafruit_VL53L0X();
Adafruit_VL53L0X vl53_4 = Adafruit_VL53L0X();

// --- Function Prototypes ---
void initializeVL53L0X(Adafruit_VL53L0X &sensor, int xshutPin, uint8_t newAddress);
void printVL53L0XResult(VL53L0X_RangingMeasurementData_t measure);
void readIRSensors();
void readMMA7361();

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

// --- MMA7361 Initialization ---
pinMode(MMA7361_SLEEP_PIN, OUTPUT);
digitalWrite(MMA7361_SLEEP_PIN, LOW); // Keep MMA7361 active (no sleep)

// --- IR Sensor Initialization ---
pinMode(IR_SENSOR_1_PIN, INPUT);
pinMode(IR_SENSOR_2_PIN, INPUT);
pinMode(IR_SENSOR_3_PIN, INPUT);
pinMode(IR_SENSOR_4_PIN, INPUT);

Serial.println("Sensor Test");
}

void loop() {
// --- Test VL53L0X Sensors ---
VL53L0X_RangingMeasurementData_t measure;

Serial.println("--------------------");

Serial.print("VL53L0X 1: ");
vl53_1.rangingTest(&measure, false); // Test sensor 1
printVL53L0XResult(measure);

Serial.print("VL53L0X 2: ");
vl53_2.rangingTest(&measure, false); // Test sensor 2
printVL53L0XResult(measure);

Serial.print("VL53L0X 3: ");
vl53_3.rangingTest(&measure, false); // Test sensor 3
printVL53L0XResult(measure);

Serial.print("VL53L0X 4: ");
vl53_4.rangingTest(&measure, false); // Test sensor 4
printVL53L0XResult(measure);

// --- Test MMA7361 Accelerometer ---
readMMA7361();

// --- Read IR Sensors ---
readIRSensors();

delay(1000);
}

// --- Helper Functions ---

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

void readIRSensors() {
Serial.println("--------------------");
Serial.println("IR Sensor Readings:");

bool ir1 = digitalRead(IR_SENSOR_1_PIN);
bool ir2 = digitalRead(IR_SENSOR_2_PIN);
bool ir3 = digitalRead(IR_SENSOR_3_PIN);
bool ir4 = digitalRead(IR_SENSOR_4_PIN);

Serial.print("IR 1: ");
Serial.print(ir1);
Serial.print(", IR 2: ");
Serial.print(ir2);
Serial.print(", IR 3: ");
Serial.print(ir3);
Serial.print(", IR 4: ");
Serial.println(ir4);
}

void readMMA7361() {
Serial.println("--------------------");
Serial.println("MMA7361 Readings:");

int x = analogRead(MMA7361_X_PIN);
int y = analogRead(MMA7361_Y_PIN);
int z = analogRead(MMA7361_Z_PIN);

Serial.print("X: ");
Serial.print(x);
Serial.print(", Y: ");
Serial.print(y);
Serial.print(", Z: ");
Serial.println(z);

// You should calibrate these values based on the MMA7361 datasheet
// and your specific setup. This is a very basic example.
float x_g = (float)(x - 512) / 102.4; // Assuming 512 is the zero-g offset, and 102.4 is the sensitivity (adjust these)
float y_g = (float)(y - 512) / 102.4;
float z_g = (float)(z - 512) / 102.4;

Serial.print("X (g): ");
Serial.print(x_g);
Serial.print(", Y (g): ");
Serial.print(y_g);
Serial.print(", Z (g): ");
Serial.println(z_g);
}
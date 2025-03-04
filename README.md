# QUANTUM: A High-Agility Autonomous Mobile Robot

**Author:** Almoulla Almaawali 
**Institution:** Sultan Qaboos University

## 1. Introduction

QUANTUM is an autonomous mobile robot platform designed for high-maneuverability and robust obstacle avoidance.  This document provides a comprehensive technical overview of the robot's hardware, software, and control systems. The platform utilizes an ESP32-S3 microcontroller, a differential drive system, and a multi-sensor array for precise environmental perception.

## 2. System Architecture

The robot's architecture is centered around a modular design, enabling future expansion and modification.  Key subsystems include:

*   **Locomotion:**  Differential drive system with two independently controlled DC motors.
*   **Sensing:**  A suite of Time-of-Flight (ToF) and Infrared (IR) sensors for distance measurement and edge detection.
*   **Processing:**  An ESP32-S3 microcontroller for real-time control, sensor data processing, and navigation algorithms.
*   **Power:**  A 7.4V LiPo battery providing power to all onboard components.
*    **Weight** Approximately 2.5KG

## 3. Hardware Components

### 3.1 Microcontroller

*   **Model:** ESP32-S3-WROOM
*   **Features:**
    *   Dual-core Xtensa LX7 processor
    *   Integrated Wi-Fi (802.11 b/g/n) and Bluetooth (v5.0, BLE)
    *   Multiple GPIO pins for peripheral interfacing
    *   12-bit ADC for analog sensor readings
    *   Sufficient SRAM and Flash memory for program storage and data logging

### 3.2 Locomotion System

*   **Motors:**  Two MG310 7.4V DC brushed motors.
    *   **Gear Ratio:** 1:20 reduction gearbox.
    *   **Stall Torque:** 1.5 KGf-cm per motor.
    *   **Encoders:** Integrated Hall effect encoders for precise closed-loop speed control.  Provides quadrature encoder output.
*   **Motor Driver:**  A dual H-bridge motor driver circuit capable of providing sufficient current (>= 2A per motor) and handling the motor's voltage requirements.  Includes protection features (e.g., overcurrent, overvoltage).
*   **Drive Configuration:**  Differential drive, enabling zero-radius turning and precise control over heading and velocity.
* **Shaft:** 5mm D shaft
*   **Kinematic Analysis:**
    *   Simulated Static Coefficient of Friction: 1.15
    *   Maximum Theoretical Acceleration: 4 m/s²
    *   Time to Reach 1.05 m/s: < 0.25 seconds (simulated)

### 3.3 Sensor Suite

*   **Time-of-Flight (ToF) Sensors:**
    *   **Model:**  VL53L0X
    *   **Quantity:** Four
    *   **Interface:** I2C
    *   **Wavelength:** 940nm (Infrared)
    *   **Field of View (FoV):** 27 degrees
    *   **Functionality:** Provides precise distance measurements to surrounding objects, enabling obstacle detection and mapping.
*   **Infrared (IR) Sensors:**
    *   **Quantity:** 4
    *   **Configuration:** Active-low output.
    *   **Functionality:** Detects abrupt changes in surface reflectivity, primarily used for edge detection and cliff avoidance.

### 3.4 Chassis

*   **Design:** Custom-designed and fabricated.
    *   **Material:** Aluminum (selected for its lightweight nature, ease of machining, and sufficient strength).
    *   **Thickness:** 2mm.
    *   **Dimensions:** 20cm x 20cm.
    *   **Fabrication Method:** CNC machining from CAD design (custom design).
    *   **Purpose:** Provides a rigid and stable platform for mounting all electronic components, motors, and sensors.

![Custom Chassis](https://github.com/oulla898/QUANTUM-/blob/3baaca86c5ad53ef21653d8983fbaddc7042e5a4/CNC-ed%20chassis%20aluminum%20.jpg?raw=true)

### 3.5 Power System

*   **Battery:** 7.4V, 1.6Ah 2S LiPo battery.
*   **Power Distribution:**  A power distribution board or circuit to regulate and distribute power to the various components (microcontroller, motor driver, sensors).  Includes appropriate voltage regulators and filtering capacitors.

## 4. Software and Control Algorithms

*   **Programming Language:** C/C++ (using the Arduino IDE & VS-code).
*   **Motor Control:**
    *   **Closed-Loop Speed Control:**  PID (Proportional-Integral-Derivative) control algorithm.
    *   **Encoder Feedback:** Utilizes Hall effect encoder data for precise speed and position control.
    *   **Motor Driver Interface:**  Software functions for controlling motor speed and direction via the motor driver.
*   **Navigation and Obstacle Avoidance:**
    *   **Sensor Fusion:**  Combines data from ToF and IR sensors to create a local representation of the environment.
    *   **Evasive Maneuver Strategy:**  Implements algorithms for detecting and avoiding obstacles, prioritizing rapid response and maintaining safe distances.
    *   **Edge Detection:**  Utilizes IR sensor readings to prevent falls from elevated surfaces.

## 5. Parts Overview
![Parts Overview](https://github.com/oulla898/QUANTUM-/blob/eec5027533dc6741bd1c5c335dc5ba3137be9a86/quantom%20parts.jpg?raw=true)

## 6. Project Status and Future Work

The QUANTUM robot is currently under development.  Future work includes:

*   Refinement of navigation algorithms.
*   Implementation of more advanced sensor fusion techniques.
*   Exploration of mapping and localization capabilities.

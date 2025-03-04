# Quantum Robot - Technical Overview

**Author:** Almoulla Almaawali
**Institution:** Sultan Qaboos University


## 1. Introduction

Quantum is a highly mobile and responsive robot designed for autonomous navigation and obstacle avoidance.  This document provides a technical overview of its key components and operational principles.  It utilizes an ESP32-S3-WROOM microcontroller, a differential drive system, and a suite of sensors for precise environmental awareness.

## 2. Design Overview

*   **Differential Drive:** Quantum employs a differential drive system with two independently controlled wheels.  This configuration allows for excellent maneuverability, including zero-radius turns.
*   **Chassis:**  A custom-designed chassis (approximately 16cm x 18cm) provides a robust and compact platform for all components.
*    **Weight** Approximately 2.5KG

## 3. Electrical and Electronic Systems

*   **Microcontroller:**  An ESP32-S3-WROOM module serves as the central processing unit.  This powerful microcontroller offers ample processing power, numerous I/O pins, and built-in Wi-Fi/Bluetooth connectivity. The 12-bit ADC is crucial for interfacing with analog sensors.
*   **Motors:** Two MG310 7.4V DC brushed motors with integrated 1:20 reduction gearboxes provide the driving force. Each motor delivers 1.5 KGf-cm of stall torque, resulting in a high level of responsiveness.  Hall effect encoders integrated into the motors provide feedback for closed-loop speed control.
*  **Motor Simulations:** simulations where done, theoretical Static Coefficient of Friction 1.15, maximum acceleration of 4 m/s2 and Reaching 1.05 m/s is less than quarter of a second.
*   **Motor Driver:** A dedicated motor driver, compatible with the MG310 motors, controls motor speed and direction.  The driver provides sufficient current capacity (around 2A per motor) and includes a built-in power supply for the microcontroller and encoders.
*   **Power Source:** A 7.4V, 1.6Ah 2S Li-Po battery provides power to the entire system.
*   **Shaft:** 5mm D shaft

## 4. Sensor Systems

*   **Time-of-Flight (ToF) Sensors:** Four VL53L0X ToF laser-ranging sensors (940nm, I2C interface) provide precise distance measurements to surrounding objects.  These sensors have a 27-degree field of view (FoV) and are less susceptible to interference than other distance sensing technologies.
*   **Infrared (IR) Sensors:** Multiple IR sensors (active-low configuration) are used for detecting sharp changes in reflectivity.

## 5. Software and Control

*   **Closed-Loop Speed Control:**  A PID (Proportional-Integral-Derivative) control algorithm, implemented in software, maintains precise motor speeds based on feedback from the Hall effect encoders.  This ensures accurate and consistent movement.
*   **Autonomous Navigation:** The robot operates autonomously, utilizing sensor data to navigate its environment.
*   **Code Strategy:**  The primary operational mode is focused on evasive maneuvers. 
*  **Edge Avoidance:**  Using input from the IR sensors, the robot detects changes in surface reflectivity.

## 6. Parts Overview
![Parts Overview](https://github.com/oulla898/QUANTUM-/blob/eec5027533dc6741bd1c5c335dc5ba3137be9a86/quantom%20parts.jpg?raw=true)

## 7. Conclusion
The project offers excellent stability with its speed and navigation system, this file describes a technical documentation of Quantum robot, encompassing its design, electrical system, sensor integration, and control strategies. The project is still in development.

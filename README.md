# 🚗 Driver Drowsiness Detection Embeded System

<!--배지-->
![License](https://img.shields.io/badge/license-MIT-blue.svg)
![Platform](https://img.shields.io/badge/platform-STM32-0A7CCF.svg)
![Arduino](https://img.shields.io/badge/Compatible-Arduino-00979D.svg)
![Android](https://img.shields.io/badge/Compatible-Android-3DDC84.svg)
![Framework](https://img.shields.io/badge/framework-TensorFlow_Lite-FF6F00.svg)
![Language](https://img.shields.io/badge/language-C%2B%2B-blue.svg)
 
## 📖 Table of Contents
1. [Project Overview](#-project-overview)
2. [Technologies & Hardware](#-technologies--hardware)
3. [System Configuration](#-system-configuration)
4. [Expected Benefits](#-expected-benefits)
5. [License](#-license)

## 📌 Project Overview
The Driver Drowsiness Detection System is an embedded system designed to analyze the driver’s face in real-time and determine drowsiness.
By implementing this system, we aim to prevent accidents caused by drowsy driving and create a safer driving environment.


## Features


## 🛠 Technologies & Hardware
- **Embedded Board**: STM32F10x
- **Camera Module**: ESP32-CAM
- **Communication Methods**:
  - ESP32-CAM ↔ STM32F Board: USART1
  - STM32F Board ↔ Bluetooth Module: USART2
  - Bluetooth Module ↔ Android: Bluetooth Communication 
- **Timer Usage**:
  - TIM2: PIR sensor input processing
  - TIM3: Vibration motor control (PWM output)
- **Drowsiness Detection Model**:
  - Dataset: NTHU Drowsy Driver Detection Dataset
  - Model Architecture: CNN
  - Framework: TensorFlow Lite (converted to TFLite for embedded implementation)
 
## ⚙ System Configuration
1. **ESP32-CAM**: Captures the driver’s face in real-time and processes the image using the TFLite model.
2. **STM32F Board**: 
   - Manages communication between ESP32-CAM, Bluetooth module, and Android Smartphone.
   - Handles PIR sensor input for detecting driver presence.
   - Controls the vibration motor using PWM output based on received drowsiness alerts.
   - //추가..
3. **Bluetooth Module**:
   - Connects to a smartphone app to display alerts and warnings.
   - Can be integrated with external alert devices (e.g., speakers, vibration motors).
4. **Android Smartphone**:
   - Receives Bluetooth alerts from STM32F Board.
   - **Automatically plays music** when a drowsiness alert is received.
5. **Vibration Motor**: Provides a physical alert when drowsiness is detected.
6. **PIR Sensor**: Detects the presence of the driver.
7. **Light Sensor**: Adjusts drowsiness detection sensitivity dynamically under varying lighting conditions.

## 🚀 Expected Benefits
- **Enhanced Driver Safety**: Prevents accidents caused by drowsy driving.
- **Integrated Alert System**: Utilizes visual, auditory, and haptic feedback for effective warnings.
- **Android Compatibility**: Enables seamless integration with Android smartphones for real-time monitoring.
- **Automatic Music Playback**: Helps keep the driver awake by playing music upon drowsiness detection.
- **Optimized Embedded System**: Uses a lightweight model for real-time analysis.

## 📜 License
MIT License

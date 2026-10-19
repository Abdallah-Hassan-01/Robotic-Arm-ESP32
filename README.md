# ESP32 3-DOF Robotic Arm 🤖

An advanced embedded systems and IoT project featuring a 3-Degree-of-Freedom (3-DOF) robotic arm powered by an ESP32 microcontroller. This project integrates dual-mode operation supporting seamless real-time cloud control and precise physical input.

## ✨ Key Features
- **IoT Cloud Integration:** Full remote telemetry and joint control via the Blynk platform over Wi-Fi.
- **Precision Manual Control:** Dual analog joysticks and a rotary encoder (KY-040) for fine-grained physical manipulation.
- **Real-Time Display:** 0.96" OLED screen interface displaying live joint angles and system status.
- **Record & Playback:** Automated path-recording functionality to store and re-execute movement sequences.
- **Power Management & Optimization:** Intelligent idle-timeout handling and dynamic CPU frequency scaling for power efficiency.

## 🛠️ Hardware Architecture
- **Microcontroller:** ESP32 (ESP-WROOM-32)
- **Actuators:** 4x Micro Servo Motors
- **Input Peripherals:** 2x Dual-Axis Joysticks, 1x Rotary Encoder
- **Display:** 0.96-inch OLED Display (I2C)
- **Power Supply:** 3.7V 1000mAh Li-Po Battery

## 📂 Repository Structure
```text
📦 ESP32-Robotic-Arm
 ┣ 📂 ESP32_Firmware/         # Arduino source code & firmware logic
 ┣ 📂 Gerber_Files/           # PCB manufacturing and layout files
 ┣ 📂 Docs/                   # Engineering design reports & documentation
 ┗ 📂 Media/                  # Circuit schematics, 3D renderings, and demo videos

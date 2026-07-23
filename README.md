# CropDrop Bot 🚜📦
**Autonomous Logistics System for the e-Yantra International Robotics Competition**

<img width="1080" height="603" alt="WhatsApp Image 2026-07-23 at 11 56 01" src="https://github.com/user-attachments/assets/2f82bdc4-3700-40dc-8f54-c4288b9cd1be" />

## 🏆 Project Overview
This repository contains the hardware designs and software stack for the "CropDrop Bot," an autonomous logistics robot. Developed for the **international e-Yantra Robotics Competition**, our system was engineered to navigate complex environments, manage payload logistics, and execute precise line-following tasks. 

Out of over 1,000 participating teams globally, our project secured a place among the **Top 30 ranked teams**.
The robot bridges low-level precision hardware with high-level autonomous decision-making. 

### Hardware Specifications
* **Microcontroller:** STM32 F103RB (ARM Cortex-M3) acting as the powerhouse for low-level execution and sensor polling.
* **Chassis & Mechanical:** Custom-modeled in Autodesk Fusion 360. The physical bot was specifically designed with a low center of gravity to ensure stability during dynamic movements and payload handling.

### Software Stack
* **Languages:** Python, C++
* **Control Systems:** We implemented a hybrid control approach using **PID controllers** combined with **Reinforcement Learning** to achieve smooth, adaptable, and highly accurate line-following behaviors under varying conditions.

## 📁 Repository Structure
```text
├── cad/                 # Fusion 360 STEP files and chassis renders
├── firmware/            # STM32 F103RB C++ source code and libraries
├── docs/                # Schematics, competition reports, and media
└── README.md

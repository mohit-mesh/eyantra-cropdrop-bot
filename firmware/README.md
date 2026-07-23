# 💻 Firmware & Low-Level Control

This directory contains the embedded C/C++ firmware developed for the STM32 F103RB microcontroller, which acts as the low-level hardware interface for the CropDrop Bot.

The firmware is responsible for executing real-time control loops, processing raw sensor data, and safely actuating the mechanical systems based on high-level commands from the ROS 2 stack.

## ⚙️ Core Features & Capabilities

The codebase utilizes the **STM32 HAL (Hardware Abstraction Layer)** to manage the following sub-systems:

*   **Closed-Loop Locomotion:** Implements a real-time **PID control loop** for high-speed, accurate line following, utilizing fast-polling ADC reads from the IR sensor arrays.
*   **Payload Actuation:** Manages precise PWM signal generation (via hardware Timers) to control the servo motors responsible for the payload dropping mechanism.
*   **ROS 2 Serial Bridge:** Establishes robust, non-blocking UART communication to receive velocity/navigation commands from the main compute node and report back current hardware states.
*   **State Machine Architecture:** The main execution loop is structured as a non-blocking state machine, ensuring deterministic response times for critical hardware interrupts.

## 📂 Directory Structure

*   **`Core/Src/`**: Contains the main application logic (`main.c`), interrupt service routines (`stm32f1xx_it.c`), and hardware initialization functions.
*   **`Core/Inc/`**: Contains the corresponding header files.
*   **`.ioc File`**: The STM32CubeMX configuration file detailing the clock tree, pinout (GPIO, UART, I2C), and timer configurations.

> **Note to Reviewers:** The core logic, including the PID implementation and serial parsing, can be found in `Core/Src/main.c`.

## 🛠️ Build & Flash Instructions

This project is configured for use with **STM32CubeIDE**.

1.  Open **STM32CubeIDE**.
2.  Select `File` > `Open Projects from File System...`
3.  Navigate to this folder and click Finish to import the project.
4.  Click the **Build (Hammer)** icon to compile the `.elf` and `.bin` files.
5.  Connect your ST-Link to the STM32 F103RB board.
6.  Click the **Debug/Run** icon to flash the firmware directly to the microcontroller.

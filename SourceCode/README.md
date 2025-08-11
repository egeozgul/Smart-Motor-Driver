# STM32 Motor Controller Firmware

## 1. Introduction

This document provides a comprehensive overview of the firmware for a sophisticated motor control system based on an STM32 microcontroller. The system is designed to offer precise control over a DC motor, featuring position and speed control, a command-line interface for configuration and monitoring, and support for both UART and I2C communication protocols.

The firmware is built upon the STM32 HAL library and integrates several peripherals, including timers for PWM signal generation and encoder reading, an ADC for current sensing, and DMA for efficient data handling.

### Key Features:

* **Dual Communication Interface:** Control and monitor the motor via both UART and I2C.
* **Advanced Motion Control:**
    * **PID Control:** A Proportional-Integral-Derivative (PID) controller for precise position holding.
    * **Trapezoidal Profiling:** Smooth acceleration and deceleration for point-to-point movements.
* **Real-time Monitoring:** Stream motor position, speed (in ticks per second), and current consumption.
* **Command-Line Interface (CLI):** An intuitive text-based command system to configure parameters and control the motor.
* **Quadrature Encoder Support:** High-resolution position feedback from the motor.
* **Current Sensing:** Onboard ADC for monitoring motor current draw.
* **DMA-driven Communication:** Efficient data transfer for both UART and I2C, minimizing CPU overhead.

## 2. Hardware Setup

The firmware is configured to work with a specific set of STM32 peripherals. The primary components are:

* **`TIM1`:** For PWM generation to drive the H-bridge.
* **`TIM3`:** For reading the quadrature encoder.
* **`ADC1`:** For current sensing.
* **`USART2`:** For UART communication (CLI).
* **`I2C1`:** For I2C slave communication.
* **`DMA`:** For handling `USART2` and `I2C1` data transfers.

## 3. Software Architecture

The firmware's architecture is event-driven and organized around a main processing loop. It initializes all peripherals, then enters an infinite loop to process incoming commands from UART or I2C, execute motor control logic (PID, Trapezoidal, etc.), and handle data logging. Interrupts are used for encoder counting and I2C communication.

## 4. Communication Protocol & API

The motor controller is operated by sending text-based commands terminated by a newline character (`\n`) over UART or I2C.

### General Commands

| Command | Argument | Description |
| :--- | :--- | :--- |
| `SETSPD` | `speed` (0-100) | Sets the motor speed (Note: This function appears to be a placeholder). |
| `SETPOS` | `position` (integer) | Sets the target position for the PID controller and enters `MOTOR_PID` mode. |
| `MOVETO` | `position` (integer) | Moves the motor to a target position using a trapezoidal profile. Enters `MOTOR_TRAPEZOID` mode. |
| `FLOAT` | None | Disengages the motor, allowing it to spin freely. |
| `BRAKE` | None | Brakes the motor by shorting the terminals through the H-bridge. |
| `GETSPD` | None | Returns the current speed of the motor in ticks per second. |
| `GETPOS` | None | Returns the current 64-bit encoder position. |
| `GET_ADC` | None | Returns the current motor current in milliamps (mA). |
| `STATUS` | None | Returns the current drive mode of the motor. |
| `RESET` | None | Resets the microcontroller. |

### Configuration Commands

| Command | Argument | Description |
| :--- | :--- | :--- |
| `SET_P` | `value` (float) | Sets the Proportional (Kp) gain for the PID controller. |
| `SET_I` | `value` (float) | Sets the Integral (Ki) gain for the PID controller. |
| `SET_D` | `value` (float) | Sets the Derivative (Kd) gain for the PID controller. |
| `GET_PID` | None | Returns the current Kp, Ki, and Kd values. |
| `SETMAX_VEL` | `value` (float) | Sets the maximum velocity for trapezoidal moves. |
| `SETMAX_ACC` | `value` (float) | Sets the maximum acceleration for trapezoidal moves. |
| `GETMAX_VEL` | None | Returns the current maximum velocity setting. |
| `GETMAX_ACC` | None | Returns the current maximum acceleration setting. |
| `LOG` | `period` (ms) | Starts streaming `pos`, `amps`, and `tps` data every `period` milliseconds. Set to 0 to disable. |
| `CONFIRM` | `0` or `1` | Disables (0) or enables (1) confirmation messages for set commands. |

### Example Usage (UART)

1.  **Set PID gains:**
    ```
    SET_P 0.005
    SET_I 0.0001
    SET_D 0.01
    ```

2.  **Move to position 50000:**
    ```
    MOVETO 50000
    ```

3.  **Start logging data every 100ms:**
    ```
    LOG 100
    ```

    *Output:*
    ```
    pos 12345,amps 150.5,tps 5000
    pos 17345,amps 152.3,tps 5000
    ...
    ```

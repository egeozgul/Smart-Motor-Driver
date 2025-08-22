# Smart Motor Driver - User Guide

## Overview
The Smart Motor Driver is a sophisticated motor control system built on STM32 microcontroller, providing precise control over DC motor position, speed, and movement profiles. This guide explains how to use the system through its ASCII command interface over UART or I2C.

## Features
* High-precision position control with configurable PID parameters
* Variable speed control with PWM (0-100%)
* Advanced trapezoidal motion profiles for smooth movement
* Real-time current monitoring with mA resolution
* Multiple operation modes:
  - Float: No power to motor (free movement)
  - Brake: Active braking with 50% PWM
  - PID: Closed-loop position control
  - Trapezoid: Smooth motion with acceleration control

## Command Interface

### Basic Commands

#### Motor State Control
* `FLOAT` - Sets motor to floating state
  - Disables both PWM channels
  - Allows free movement of motor shaft
  - Useful for manual positioning or power saving

* `BRAKE` - Engages motor brake
  - Sets both PWM channels to 50% duty cycle
  - Creates active braking effect
  - Holds position against external forces

#### Status Commands
* `GETPOS` - Returns current motor position
  - Reports absolute encoder position
  - 64-bit counter value
  - Maintains position through power cycles

* `GETSPD` - Returns current motor speed
  - Reports real-time velocity
  - Calculated from encoder counts
  - Updates continuously

* `STATUS` - Shows current operating mode
  - Reports: FLOAT/BRAKE/PID/TRAPEZOID
  - Indicates active control method
  - Useful for debugging

### Motion Control

#### Position Commands
* `SETPOS <value>` - Set target position (PID mode)
  - Immediate position command
  - Uses PID control for positioning
  - Value is in encoder counts
  - Example: `SETPOS 1000`

* `MOVETO <value>` - Move to position using trapezoidal profile
  - Smooth motion with controlled acceleration
  - Automatically calculates optimal profile
  - Prevents mechanical stress
  - Example: `MOVETO 5000`

#### Speed Control
* `SETSPD <0-100>` - Set motor speed
  - Direct PWM duty cycle control
  - Range: 0% (stop) to 100% (full speed)
  - Linear speed control
  - Example: `SETSPD 75`

### PID Control

#### Tuning Commands
* `SET_P <value>` - Set proportional gain
  - Controls position error response
  - Higher values: faster response
  - Lower values: smoother motion
  - Example: `SET_P 0.001`

* `SET_I <value>` - Set integral gain
  - Eliminates steady-state error
  - Higher values: better position holding
  - Lower values: less oscillation
  - Example: `SET_I 0.0001`

* `SET_D <value>` - Set derivative gain
  - Dampens oscillations
  - Higher values: more damping
  - Lower values: faster response
  - Example: `SET_D 0.00001`

* `GET_PID` - Display current PID values
  - Shows all three parameters
  - Format: "Kp <p>,Ki <i>,Kd <d>"
  - Useful for tuning verification

### Motion Profile Settings

#### Velocity and Acceleration Control
* `SETMAX_VEL <value>` - Set maximum velocity
  - Limits peak velocity in trapezoidal profile
  - Units: encoder counts per second
  - Default: 20.0
  - Example: `SETMAX_VEL 15.5`

* `SETMAX_ACC <value>` - Set maximum acceleration
  - Controls acceleration/deceleration rate
  - Units: encoder counts per second²
  - Default: 0.05
  - Example: `SETMAX_ACC 0.03`

#### Profile Monitoring
* `GETMAX_VEL` - Display maximum velocity
  - Shows current velocity limit
  - Used for profile verification

* `GETMAX_ACC` - Display maximum acceleration
  - Shows current acceleration limit
  - Used for profile verification

### Monitoring and Diagnostics

#### Current Sensing
* `GET_ADC` - Read current sensor
  - Returns current in milliamps
  - High-precision ADC measurement
  - Resolution: 12-bit (4096 steps)
  - Range: 0 to (3.3V/1.5kΩ)

#### Data Logging
* `LOG <period>` - Set logging period
  - Sets update frequency for data output
  - Period in milliseconds
  - Example: `LOG 100` (10Hz update rate)

#### System Feedback
* `CONFIRM <0/1>` - Enable/disable command confirmations
  - 1: Enable command acknowledgments
  - 0: Disable command acknowledgments
  - Useful for debugging
  - Example: `CONFIRM 1`

## Usage Examples

1. Basic Position Control:
   ```
   SETPOS 1000    # Move to position 1000
   GETPOS         # Check current position
   ```

2. Smooth Motion Profile:
   ```
   SETMAX_VEL 20  # Set max velocity
   SETMAX_ACC 0.5 # Set max acceleration
   MOVETO 5000    # Move to position with trapezoidal profile
   ```

3. PID Tuning:
   ```
   SET_P 0.001    # Set proportional gain
   SET_I 0.0001   # Set integral gain
   SET_D 0.00001  # Set derivative gain
   GET_PID        # Verify settings
   ```

## Safety Features
* Current monitoring through ADC
* Software limits on velocity and acceleration
* Emergency stop through BRAKE command
* Motor float mode for manual positioning

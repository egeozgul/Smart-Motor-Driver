# Smart Motor Driver Documentation

## Overview
The Smart Motor Driver is a high-precision motor control system based on STM32C0 microcontroller, featuring PID control, trapezoidal motion profiles, and multiple communication interfaces.

## Documentation Structure

### [User Guide](USER_GUIDE.md)
- Getting started guide
- Command interface tutorial
- Usage examples
- Troubleshooting guide
- No C code, focused on usage

### [Technical Reference](REFERENCE.md)
- Hardware specifications
- ASCII command protocol
- Motion control commands
- PID tuning parameters
- Motion profile settings
- Monitoring commands
- System commands

### [Source Code Documentation](SOURCECODE.md)
- Core components implementation
- Motion profile generation
- Communication interfaces
- Hardware interfaces
- Data structures
- C code explanations

## Quick Links

### Common Tasks
- [Basic Motor Control](USER_GUIDE.md#basic-motor-control)
- [PID Tuning Guide](USER_GUIDE.md#pid-tuning)
- [Command Reference](REFERENCE.md#ascii-command-protocol)
- [Implementation Details](SOURCECODE.md#core-components)

### Key Features
- Position and speed control
- Trapezoidal motion profiles
- PID control with anti-windup
- Current monitoring
- UART and I2C interfaces
- 64-bit position tracking

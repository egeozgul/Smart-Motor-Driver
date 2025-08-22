# Smart Motor Driver - Technical Reference

## Technical Specifications

### Hardware Configuration
* **Microcontroller**: STM32C0 series
* **PWM Frequency**: Timer1 based
* **Encoder Resolution**: Quadrature encoding (4x)
* **ADC Resolution**: 12-bit (0-4095 steps)
* **Communication**:
  - UART: DMA-based, configurable baud rate
  - I2C: Slave mode, 7-bit addressing

### Control Parameters
* **Position Range**: 64-bit signed (-2^63 to 2^63-1)
* **PID Update Rate**: ~1kHz
* **Current Sensing**:
  - Resolution: 0.8mA (3.3V/4095/1.5kΩ)
  - Range: 0-2.2mA

## ASCII Command Protocol

### Motion Control Commands

#### SETPOS
- **Format**: `SETPOS <position>`
- **Parameters**: 64-bit signed integer
- **Response**: "Pos <value>"
- **Description**: Sets target position in PID mode
- **Example**: `SETPOS 1000`
- **Behavior**:
  - Immediate position command
  - Uses current PID settings
  - No acceleration limiting

#### MOVETO
- **Format**: `MOVETO <position>`
- **Parameters**: 64-bit signed integer
- **Response**: "Moving to <value>"
- **Description**: Moves to position using trapezoidal profile
- **Example**: `MOVETO 5000`
- **Profile Types**:
  1. Triangular (short moves)
     - When distance < 2 * acceleration_distance
     - No constant velocity phase
  2. Trapezoidal (long moves)
     - Acceleration phase
     - Constant velocity phase
     - Deceleration phase
- **Timing**:
  - T_accel = V_max / A_max
  - T_const = (distance - 2*D_accel) / V_max

#### SETSPD
- **Format**: `SETSPD <speed>`
- **Parameters**: 0-100 (percentage)
- **Response**: "Speed <value>%"
- **Description**: Sets motor speed
- **Example**: `SETSPD 50`

### State Control Commands

#### FLOAT
- **Format**: `FLOAT`
- **Parameters**: None
- **Response**: None
- **Description**: Disables motor power
- **Effects**:
  - Sets both PWM channels to 0
  - Disables H-bridge outputs
  - Allows free shaft rotation
  - Minimal power consumption
- **Use Cases**:
  - Manual positioning
  - Emergency stop
  - Power saving mode
  - System initialization

#### BRAKE
- **Format**: `BRAKE`
- **Parameters**: None
- **Response**: None
- **Description**: Engages motor brake
- **Effects**:
  - Sets both PWM channels to 50%
  - Creates active braking torque
  - Holds position against external forces
  - Higher power consumption
- **Use Cases**:
  - Emergency stop
  - Position holding
  - Load testing
  - Safety lockout

### PID Control Commands

#### SET_P/SET_I/SET_D
- **Format**: `SET_P <value>`, `SET_I <value>`, `SET_D <value>`
- **Parameters**: Float value (scientific notation supported)
- **Response**: Confirms new value
- **Description**: Sets PID parameters
- **Examples**:
  ```
  SET_P 0.001   # Moderate response
  SET_P 0.0001  # Slower, smoother response
  SET_I 0.0001  # Light integral action
  SET_D 0.00001 # Light damping
  ```
- **Parameter Effects**:
  - P (Proportional):
    - Range: 0.0001 to 0.01
    - Controls position error response
    - Higher: Faster but may oscillate
    - Lower: Smoother but slower
  - I (Integral):
    - Range: 0.0 to 0.001
    - Eliminates steady-state error
    - Anti-windup limited to ±100
  - D (Derivative):
    - Range: 0.0 to 0.0001
    - Provides damping
    - Reduces overshoot

#### GET_PID
- **Format**: `GET_PID`
- **Response**: "Kp <p>,Ki <i>,Kd <d>"
- **Description**: Returns current PID values
- **Output Format**: Scientific notation, 7 decimal places
- **Default Values**:
  ```
  Kp = 0.0001
  Ki = 0.0
  Kd = 0.0
  ```

### Motion Profile Commands

#### SETMAX_VEL
- **Format**: `SETMAX_VEL <velocity>`
- **Parameters**: Float velocity value
- **Description**: Sets maximum velocity for profiles
- **Default**: 20.0

#### SETMAX_ACC
- **Format**: `SETMAX_ACC <acceleration>`
- **Parameters**: Float acceleration value (0.01-1.0)
- **Description**: Sets maximum acceleration for profiles
- **Default**: 0.05
- **Effects**:
  - Limits acceleration in trapezoidal profiles
  - Affects move time calculation
  - Influences jerk and smoothness
- **Calculation**:
  ```
  D_accel = (V_max * V_max) / (2 * A_max)
  T_accel = V_max / A_max
  ```

### Monitoring Commands

#### GETPOS
- **Format**: `GETPOS`
- **Response**: `Pos <value>`
- **Description**: Returns encoder position
- **Features**:
  - 64-bit signed integer
  - No overflow issues
  - Quadrature decoded (4x resolution)
  - Updates at ~1kHz

#### GETSPD
- **Format**: `GETSPD`
- **Response**: `Spd <value>`
- **Description**: Returns motor speed
- **Features**:
  - Units: counts/second
  - Filtered for noise reduction
  - Sign indicates direction
  - Updates at ~1kHz

#### GET_ADC
- **Format**: `GET_ADC`
- **Response**: `ADC <raw>,<mA>`
- **Description**: Returns current sensing data
- **Features**:
  - Raw ADC value (0-4095)
  - Calculated current in mA
  - 12-bit resolution
  - ~0.8mA per step

#### STATUS
- **Format**: `STATUS`
- **Response**: Current mode (FLOAT/BRAKE/PID/TRAPEZOID)
- **Description**: Returns drive state

### System Commands

#### LOG
- **Format**: `LOG <period>`
- **Parameters**: Integer period value
- **Description**: Sets data logging period

#### CONFIRM
- **Format**: `CONFIRM <0/1>`
- **Parameters**: 0 (off) or 1 (on)
- **Description**: Enables/disables command confirmations

## Features Reference

### Motion Profiles
- Trapezoidal velocity profile
- Automatic profile selection (triangular/trapezoidal)
- Configurable velocity and acceleration limits
- Real-time trajectory computation

### Position Control
- 64-bit position counter
- PID control with anti-windup
- Configurable gains
- Encoder-based feedback

### Current Monitoring
- ADC-based current sensing
- 12-bit resolution
- 1.5kΩ sense resistor
- Real-time measurement

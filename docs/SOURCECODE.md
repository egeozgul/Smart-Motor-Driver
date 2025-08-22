# Smart Motor Driver - Source Code Documentation

## Core Components

### Motor Control System

#### PWM Control
* **Function**: `UpdatePWMDutyCycle(uint32_t channel, uint32_t value)`
* **Purpose**: Controls motor speed and direction using PWM channels
* **Implementation**:
  ```c
  // Configure PWM channel
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = value;          // PWM duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  // Start PWM output
  HAL_TIM_PWM_Start_optimized(&htim1, channel);
  ```
* **Usage**:
  - Channel 3: Forward direction
  - Channel 4: Reverse direction
  - Value range: 0 to Timer Period

#### Encoder Interface
* **Function**: `get_full_encoder_count()`
* **Purpose**: Reads absolute motor position
* **Implementation**:
  ```c
  int64_t count = full_encoder_count + 
                  __HAL_TIM_GET_COUNTER(&htim3) - 65536;
  ```
* **Features**:
  - 64-bit position counter
  - Overflow handling
  - High-resolution tracking

#### PID Controller
* **Function**: `update_PID()`
* **Purpose**: Closed-loop position control
* **Key Variables**:
  ```c
  float Kp = 0.0001;  // Proportional gain
  float Ki = 0.0;     // Integral gain
  float Kd = 0.0;     // Derivative gain
  #define INTEGRAL_MAX 100  // Anti-windup limit
  ```
* **Algorithm**:
  1. Position error calculation
  2. Integral accumulation with anti-windup
  3. Derivative computation
  4. PWM output generation

### Motion Profiles

#### Trapezoidal Profile Generator
* **Initialization**: `initializeTrajectory(int64_t start, int64_t target)`
* **Implementation**:
  ```c
  // Calculate acceleration distance
  D_accel = (V_max * V_max) / (2.0f * A_max);
  
  if (totalDistance < 2 * D_accel) {
      // Triangular profile
      D_accel = totalDistance / 2;
      T_accel = custom_sqrtf(2.0f * D_accel / A_max);
      T_const = 0.0f;
  } else {
      // Trapezoidal profile
      T_accel = V_max / A_max;
      T_const = (totalDistance - 2 * D_accel) / V_max;
  }
  ```
* **Profile Parameters**:
  - `V_max`: Maximum velocity (default: 20.0)
  - `A_max`: Maximum acceleration (default: 0.05)
  - `T_accel`: Acceleration/deceleration time
  - `T_const`: Constant velocity time

#### Profile Update
* **Function**: `updateTrajectory(float dt)`
* **Phases**:
  1. Acceleration: Quadratic position increase
  2. Constant velocity: Linear position change
  3. Deceleration: Inverted quadratic
* **Implementation**:
  ```c
  if (currentTime <= T_accel) {
      // Acceleration phase
      setpointPos = startPos + direction * 
          (int64_t)((A_max * currentTime * currentTime) / 2.0f);
  } else if (currentTime <= T_accel + T_const) {
      // Constant velocity phase
      setpointPos = startPos + direction * 
          (int64_t)(V_max * (currentTime - T_accel));
  }
  ```

### Communication Interfaces

#### UART Interface
* **Configuration**:
  ```c
  #define DMA_BUFFER_SIZE 2048
  uint8_t rxBuffer[DMA_BUFFER_SIZE];
  ```
* **Features**:
  - DMA circular buffer for efficient reception
  - Non-blocking command processing
  - Automatic buffer wraparound

* **Command Processing**:
  ```c
  void ProcessCommand(uint8_t *buffer, int bufferSize, 
                     DMA_HandleTypeDef *hdmarx, 
                     uint16_t *lastParsedIndex)
  ```

#### I2C Interface
* **Buffer Management**:
  ```c
  #define I2C_BUFFER_SIZE 64
  uint8_t i2cTxBuffer[I2C_BUFFER_SIZE];
  ```
* **Features**:
  - Slave mode with 7-bit addressing
  - Interrupt-driven data transmission
  - Cyclic buffer with overflow protection

* **Data Handling**:
  ```c
  void I2C_IRQHandler_User(I2C_HandleTypeDef *hi2c)
  ```

### Command Processing

#### Command Parser
* **Function**: `handleCommand(uint8_t *buffer, int bufferSize, uint16_t commandIndex)`
* **Buffer Processing**:
  ```c
  #define MAX_COMMAND_LENGTH 25
  char command[MAX_COMMAND_LENGTH];
  // Extract command from circular buffer
  for (int i = 0; i < MAX_COMMAND_LENGTH; i++) {
      int buffer_i = (commandIndex - i);
      if (buffer_i < 0)
          buffer_i = bufferSize - buffer_i;
      command[MAX_COMMAND_LENGTH - i - 1] = buffer[buffer_i];
  }
  ```

#### Command Parser Components
* **String Functions**:
  ```c
  int strcmp_embedded(const char *str1, const char *str2)
  int atoi_embedded(const char *str)
  ```
* **Command Format**: `COMMAND VALUE\n`
* **Error Handling**:
  - Buffer overflow protection
  - Invalid command detection
  - Value range validation

### Hardware Interfaces

#### ADC Interface
* **Function**: `readADCValue()`
* **Implementation**:
  ```c
  uint32_t readADCValue() {
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
          uint32_t adcValue = HAL_ADC_GetValue(&hadc1);
      }
      HAL_ADC_Stop(&hadc1);
      return adcValue;
  }
  ```
* **Current Calculation**:
  ```c
  float Vipropi = ((float)adc_value * (3.3f/4095.0f));
  float Ipropi = Vipropi/1.5f;  // Current in mA
  ```
* **Features**:
  - 12-bit resolution (0-4095)
  - Voltage reference: 3.3V
  - Current sense resistor: 1.5kΩ
  - Polling mode with timeout

#### Timer Configuration
* **PWM Timer (TIM1)**:
  - Channels 3,4: H-bridge control
  - Complementary outputs
  - Dead-time insertion

* **Encoder Timer (TIM3)**:
  - 16-bit counter
  - Quadrature decoding
  - Overflow handling for 64-bit position

### Key Data Structures
* **Cyclic Buffer**:
  ```c
  uint8_t i2cTxBuffer[I2C_BUFFER_SIZE];
  int head;  // Write position
  int tail;  // Read position
  int is_full;
  ```

### Important Constants
```c
#define DMA_BUFFER_SIZE 2048
#define I2C_BUFFER_SIZE 64
#define INTEGRAL_MAX 100
#define MAX_COMMAND_LENGTH 25
```

### Drive Modes
```c
#define MOTOR_FLOAT     0  // No power to motor
#define MOTOR_BRAKE     1  // Active braking
#define MOTOR_PID       2  // Position control
#define MOTOR_TRAPEZOID 3  // Profile motion
```

## Implementation Details

### PID Control Loop
```c
void update_PID() {
    int64_t currentPos = get_full_encoder_count();
    int64_t error = setpointPos - currentPos;
    integral += error;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);
    int64_t derivative = error - previousError;
    double output = (Kp * error + Ki * integral + Kd * derivative)/10;
    // Apply output to PWM channels
}
```

### Trapezoidal Profile Generation
```c
void initializeTrajectory(int64_t start, int64_t target) {
    totalDistance = abs(target - start);
    D_accel = (V_max * V_max) / (2.0f * A_max);
    
    if (totalDistance < 2 * D_accel) {
        // Triangular profile
        D_accel = totalDistance / 2;
        T_accel = sqrt(2.0f * D_accel / A_max);
        T_const = 0;
    } else {
        // Trapezoidal profile
        T_accel = V_max / A_max;
        T_const = (totalDistance - 2 * D_accel) / V_max;
    }
}
```

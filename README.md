# Smart DC Motor Driver (with Built-in Feedback Controller)

<p align="center">
  <img src="pcb.png" alt="Encoder Motor Driver - Front View" width="70%" height="auto"/>
</p>

The **Smart Encoder Motor Driver** is designed to control DC motors with magnetic or optical encoders. It features a built-in feedback controller and H-bridge, allowing precise motor power adjustment based on commands from a master device. Powered by the **STM32C0 microcontroller** and equipped with a **PID feedback system**, this driver ensures reliable and efficient motor control.

## Key Features
- **Built-in Feedback Controller**: High-frequency PID control for precise motor operation.
- **Digital Communication**: Supports ASCII modes for setting motor target position, adjusting PID coefficients, receiving torque feedback, and more.
- **Voltage Range**: 5V to 45V.
- **Current Capacity**: Up to 4.1 amps.
- **Communication**: Supports I2C and UART protocols.
- **Expandable Design**: Qwiic connectors allow daisy-chaining of up to 256 units on a single I2C port.

## Motion Plots
A custom web-based UART plotter is used for live plotting of motor motion parameters. The plots display:
1. Position
2. Velocity
3. Current drawn by the motor

![scr](https://github.com/user-attachments/assets/ec2e79d4-fb35-4699-954e-250158fb1edc)

## Custom Live Plotter
A custom live plotter is built for monitoring the motor speed, acceleration, and current draw.
Check out the [Live UART Plotter](https://egeozgul.github.io/live-uart/) to plot variables sent over uart for your project.
It plot infinetly many variables with customized styling for each plot.

## Board Overview (First Iteration)
<p align="center">
  <img src="imageC.jpg" alt="Encoder Motor Driver - Front View" width="50%" height="auto"/>
</p>
<p align="center">
  <img src="imageA.jpg" alt="Encoder Motor Driver - Front View" width="50%" height="auto"/>
</p>

The **PID feedback control system** ensures precise and stable motor control, making it suitable for robotics, automation, and other applications requiring high precision.

## Connectivity and Expansion
The motor driver is designed for scalability, using **Qwiic connectors** for easy daisy-chaining. This allows control of a large number of motors, making it ideal for projects requiring extensive motor coordination.

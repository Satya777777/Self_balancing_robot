Self-Balancing Robot (Arduino + MPU9250 + PID Control)

This project implements a simple self-balancing robot using an MPU9250 IMU sensor and PID control on an Arduino-compatible board. The robot continuously adjusts motor speed and direction to maintain an upright position by keeping the tilt angle close to 0°.

🚀 Features

MPU9250 IMU integration for real-time angle measurement (using complementary filter).

PID control loop for maintaining balance.

Motor driver (L298N / similar) for driving two DC motors.

Serial debugging with live angle, error, and PID output values.

🛠️ Hardware Requirements

Arduino (UNO, Mega, Nano, etc.)

MPU9250 (I2C interface)

L298N motor driver (or similar dual H-bridge driver)

2 DC gear motors + wheels

Battery pack (7.4V–12V recommended)

⚡ Pin Connections
MPU9250 (I2C)

VCC → 5V

GND → GND

SDA → A4 (Arduino UNO)

SCL → A5 (Arduino UNO)

Motor Driver (L298N)

Motor A: enA → D5, in1 → D6, in2 → D7

Motor B: enB → D9, in3 → D10, in4 → D11

Motor power (12V) → Battery

Driver VCC → 12V supply

Driver GND → Arduino GND

⚙️ How It Works

The MPU9250 reads accelerometer and gyroscope data.

A complementary filter fuses the data to estimate the robot’s tilt angle.

The PID controller calculates a correction value based on:

Proportional (Kp): immediate error

Integral (Ki): accumulated error

Derivative (Kd): rate of change of error

The correction value controls the speed and direction of both motors to balance the robot.

📊 PID Tuning

You can adjust the PID constants in the code for stable balancing:

double Kp = 25.0;   // Proportional gain
double Ki = 1.0;    // Integral gain
double Kd = 2.0;    // Derivative gain


👉 Start by tuning Kp, then Kd, and finally fine-tune with Ki.

🖥️ Serial Output

The program prints debugging values at 115200 baud:

Angle: -2.15    Error: 2.15    PID Output: 53.4

▶️ Getting Started

Install the MPU9250 library in Arduino IDE.

Upload the code to your Arduino board.

Power the robot with a battery pack.

Place the robot upright and watch it attempt to balance.

📌 Notes

Ensure motors and wheels are well-aligned for stable balance.

Use a sturdy battery (Li-ion or Li-Po) for consistent power.

Proper PID tuning is crucial for smooth balancing.

You may need to reverse motor wiring if the robot falls in the wrong direction.

📷 Future Improvements

Add Bluetooth/Wi-Fi control for remote monitoring.

Implement Kalman Filter for more accurate angle estimation.

Design a 3D-printed chassis for stability.
# Project Arceus

Project Arceus is an advanced quadcopter flight controller implementation using Mbed OS, designed for the NUCLEO-F411RE board. It features precise flight control through MPU6050 sensor fusion, PID control, and PPM signal processing.

## Features

- **Sensor Fusion**: Implements Mahony filter for accurate orientation estimation using MPU6050 (gyroscope and accelerometer)
- **PID Control**: Three separate PID controllers for pitch, roll, and yaw control
- **PPM Signal Processing**: Handles RC receiver input with up to 8 channels
- **Motor Control**: Precise PWM control for all four motors
- **Safety Features**: 
  - Arming system to prevent accidental motor activation
  - Motor failsafe when signal is lost
  - Audio feedback through buzzer
  - Stick override for emergency control

## Hardware Requirements

- NUCLEO-F411RE Development Board
- MPU6050 IMU Sensor
- 4x Brushless Motors
- RC Receiver with PPM output
- Buzzer (optional)

## Pin Configuration

- **I2C for MPU6050**:
  - SCL: PB_8
  - SDA: PB_9

- **Motor Outputs (PWM)**:
  - Motor1 (Front Left): PB_13
  - Motor2 (Front Right): PA_10
  - Motor3 (Rear Right): PB_6
  - Motor4 (Rear Left): PB_5

- **Other Connections**:
  - PPM Input: PA_0
  - LED: PC_13
  - Buzzer: PA_15
  - UART (Debug): PA_2 (TX), PA_3 (RX)

## Building the Project

1. Clone the repository:
   ```bash
   git clone https://github.com/ShekharShwetank/Project-Arceus.git
   cd Project-Arceus
   ```

2. Deploy Mbed OS:
   ```bash
   mbed deploy
   ```

3. Compile the project:
   ```bash
   mbed compile -t GCC_ARM -m NUCLEO_F411RE
   ```

## Flight Control Parameters

The flight controller uses carefully tuned PID parameters:

### Pitch Control
- Kp: 0.33
- Ki: 0.003
- Kd: 0.06
- Output Limits: -0.10 to 0.10

### Roll Control
- Kp: 0.31
- Ki: 0.003
- Kd: 0.07
- Output Limits: -0.9 to 0.9

### Yaw Control
- Kp: 0.3
- Ki: 0.001
- Kd: 0.08
- Output Limits: -0.04 to 0.06

## Flight Controls

- Channel 1: Roll control
- Channel 2: Pitch control
- Channel 3: Throttle
- Channel 4: Yaw control
- Channel 5: Arm/Disarm

## Safety Features

1. **Arming System**: The quadcopter must be armed using Channel 5 before motors will respond
2. **Throttle Safety**: Motors won't spin if throttle is outside safe range (900-2100 μs)
3. **Stick Override**: Moving both pitch and roll to maximum triggers emergency override
4. **Drift Compensation**: Implements yaw drift compensation when stationary
5. **Audio Feedback**: Buzzer provides status indication tones

## Project Images

### Component Layout
![Components](resources/Components%20.jpg)

### Drone Views
![Top View](resources/TOP%20VIEW.jpg)
![Front View](resources/Front%20View.jpg)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Author

- **Shekhar_Shwetank** 
- **Subhojeet Sural**
- **Arnav Sharma**
- **Sneha Sharma**

## Acknowledgments

- Based on Mbed OS
- Uses MPU6050 sensor fusion algorithms
- Implements Mahony filter for orientation estimation
# rover-model


### 1. Project Overview
Goal: Build a 6-wheeled all-terrain rover with a 5-axis robotic manipulator capable of autonomous navigation, mapping (SLAM), and remote manipulation. Platform: ROS Noetic (Ubuntu 20.04). Compute Architecture:

- Master (PC): Handles heavy computation (SLAM, Path Planning, MoveIt, Rviz).
- Slave (Raspberry Pi 4): Handles hardware interfacing (Camera, Arduinos, Diagnostics).

#

### 2. Hardware Topology
```
Subsystem       Components                               Connection                           Function

Mobility,       6x 12V DC Motors (83 RPM) + Encoders,    Arduino Mega #1 (PWM/Interrupts),    Traction & Odometry
Steering,       4x 40kg Servos (Corners),                Arduino Mega #1,                     Ackerman/Crab Steering
Power,          "3x DRI0002 Drivers, INA219 Sensor",     Arduino Mega #1 (I2C),               Motor Drive & Battery Monitoring
Sensing,        MPU6050 IMU,                             Arduino Mega #1 (I2C),               Orientation & Acceleration
Vision,         Kinect 360 RGB-D Camera,                 Raspberry Pi 4 (USB),                Visual Odometry & Obstacle Avoidance
Manipulation,   5x NEMA17 Steppers + A4988 Drivers,      Arduino Mega #2,                     5-DOF Robotic Arm Control
```
#

### 3. Software Stack

#### Level 1: Firmware (Arduino)
- ``rover_base.ino`` : Controls the 6 DC motors and 4 Servos. Publishes raw encoder ticks and IMU data to ROS.
- ``rover_arm.ino`` : Controls the 5 stepper motors. Includes a safety ENABLE feature (Pin 32) and homing calibration logic.

Level 2: Drivers & Network (Raspberry Pi)rover_drivers.launch: The startup script that launches the Kinect drivers (compressed for Wi-Fi), establishes serial links to both Arduinos, and starts the system health monitor.health_monitor.py: A diagnostic node reporting Battery Voltage, CPU Temperature, and Wi-Fi Signal Strength.

Level 3: Perception & Navigation (PC)RTAB-Map: Performs SLAM (Simultaneous Localization and Mapping) using the Kinect's RGB-D stream for Visual Odometry.Move Base: Uses the map to plan paths.Global Planner: Finds the optimal route across the known map.Local Planner (DWA): Avoids dynamic obstacles using a "Fake Laser Scan" generated from the depth camera.kinematics.py: Converts the Navigation stack's simple velocity commands (cmd_vel) into complex 6-wheel speeds and 4-corner steering angles.

Level 4: Manipulation (PC)MoveIt: Calculates Inverse Kinematics (IK) to move the arm's end-effector to a specific X, Y, Z coordinate without self-collision.arm_bridge.py: A translation node that takes MoveIt's abstract joint angles and converts them into specific stepper motor step counts for Arduino #2.

Level 5: Control Interfacesautonomous_mission.py: A "Mission Commander" script that sequences tasks: Undock Arm $\to$ Navigate to Coordinate $\to$ Deploy Arm $\to$ Dig $\to$ Return Home.Web Dashboard: An HTML5/JS interface hosted on the robot, allowing control via any browser (Phone/Laptop). It features a live camera feed, virtual joystick, arm sliders, and battery stats.

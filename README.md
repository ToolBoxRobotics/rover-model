# rover-model


## 1. Install Dependencies 
#### (On PC)
We need the navigation stack and a tool to convert the Kinect's 3D depth into a 2D "laser scan" for simpler obstacle avoidance.
```bash
sudo apt install ros-noetic-move-base ros-noetic-dwa-local-planner ros-noetic-depthimage-to-laserscan ros-noetic-map-server
```

#### (On RPI)
```bash
sudo apt install ros-noetic-rosserial-python ros-noetic-freenect-launch
```


## How to Run It

#### On Raspberry Pi (SSH terminal 1):
```Bash
roslaunch rover_pkg rover_drivers.launch
```
#### On PC (Terminal 1):

``` Bash
# Ensure network compression is enabled for image transport
rosparam set /camera/rgb/image_rect_color/compressed/jpeg_quality 50
rosparam set /camera/depth_registered/image_raw/compressedDepth/depth_max 10.0

roslaunch rover_control rover_slam.launch
```


#
#
#

### 1. System Architecture & Network Setup
#### Concept:
* Master (PC): Runs computationally heavy tasks (Rviz, Navigation Stack, MoveIt for the arm, SLAM).
* Slave (RPi 4): Runs hardware drivers (rosserial, Kinect drivers) and publishes sensor data.

#### Network Configuration (On both machines):
1. Connect both to the same Wi-Fi/Ethernet.
2. Edit ``/etc/hosts`` to add each other's IP addresses and hostnames.
3. On PC (``.bashrc``):
```bash
export ROS_MASTER_URI=http://PC_IP:11311
export ROS_HOSTNAME=PC_IP
```
4. On RPi (``.bashrc``):
```bash
export ROS_MASTER_URI=http://PC_IP:11311
export ROS_HOSTNAME=RPI_IP
```
#

### 2. Arduino Mega #1: The Mobile Base
#### Responsibilities: 
6x Drive Motors, 6x Encoders, 4x Steering Servos, IMU, Power Sensor.

#### Dependencies:

* ``ros_lib`` (Install via Arduino Library Manager or generate using ``rosrun rosserial_arduino make_libraries.py`` .)
* ``PinChangeInterrupt`` (Required because Mega has only 6 hardware interrupts, but you have 12 encoder channels).
* `` Wire.h``, ``Servo.h``, ``Adafruit_MPU6050``, ``Adafruit_INA219``.

Sketch (``rover_base.ino``):

#

### 3. Arduino Mega #2: The Robotic Arm
#### Responsibilities: 
Control 5 Stepper motors (NEMA17 + A4988), read 5 Limit Switches.

Dependencies: ``AccelStepper`` library.
Sketch (``rover_arm.ino``):

#

### 4. ROS Environment (RPi Side)
You need a launch file to start the camera and the serial connections to the Arduinos.

#### Prerequisites:
```bash
sudo apt install ros-noetic-rosserial-python ros-noetic-freenect-launch
```
``rover_drivers.launch``:

#

### 5. ROS Environment (PC Side)
The PC handles the "brains": converting velocity commands to wheel speeds (Inverse Kinematics) and Navigation.

### Inverse Kinematics Node (Python):

Since you have 6 wheels and 4 steering servos, simple differential drive (skid steer) logic isn't enough. You need a node that subscribes to ``cmd_vel`` (geometry_msgs/Twist) and publishes the ``rover/drive_cmd`` array expected by Arduino #1.

File: ``rover_control/scripts/kinematics.py`` (Simplified snippet):






#

### 1. Project Overview
Goal: Build a 6-wheeled all-terrain rover with a 5-axis robotic manipulator capable of autonomous navigation, mapping (SLAM), and remote manipulation. Platform: ROS Noetic (Ubuntu 20.04). Compute Architecture:

- Master (PC): Handles heavy computation (SLAM, Path Planning, MoveIt, Rviz).
- Slave (Raspberry Pi 4): Handles hardware interfacing (Camera, Arduinos, Diagnostics).

#

### 2. Hardware Topology
```txt
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

#### Level 2: Drivers & Network (Raspberry Pi)
- ``rover_drivers.launch`` : The startup script that launches the Kinect drivers (compressed for Wi-Fi), establishes serial links to both Arduinos, and starts the system health monitor.
- ``health_monitor.py`` : A diagnostic node reporting Battery Voltage, CPU Temperature, and Wi-Fi Signal Strength.

#### Level 3: Perception & Navigation (PC)
- RTAB-Map: Performs SLAM (Simultaneous Localization and Mapping) using the Kinect's RGB-D stream for Visual Odometry.
- Move Base: Uses the map to plan paths.
  - Global Planner: Finds the optimal route across the known map.
  - Local Planner (DWA): Avoids dynamic obstacles using a "Fake Laser Scan" generated from the depth camera.
- ``kinematics.py`` : Converts the Navigation stack's simple velocity commands (cmd_vel) into complex 6-wheel speeds and 4-corner steering angles.

#### Level 4: Manipulation (PC)
- MoveIt: Calculates Inverse Kinematics (IK) to move the arm's end-effector to a specific X, Y, Z coordinate without self-collision.
- ``arm_bridge.py`` : A translation node that takes MoveIt's abstract joint angles and converts them into specific stepper motor step counts for Arduino #2.

#### Level 5: Control Interfaces
- ``autonomous_mission.py`` : A "Mission Commander" script that sequences tasks: Undock Arm $\to$ Navigate to Coordinate $\to$ Deploy Arm $\to$ Dig $\to$ Return Home.
- Web Dashboard: An HTML5/JS interface hosted on the robot, allowing control via any browser (Phone/Laptop). It features a live camera feed, virtual joystick, arm sliders, and battery stats.

#

### 4. Final Directory Structure
This is the workspace organization created for the project.
```txt
~/catkin_ws/src/
├── rover_pkg/                       # [RPi] HARDWARE INTERFACE
│   ├── firmware/                    # Arduino Sketches
│   │   ├── rover_base/rover_base.ino
│   │   └── rover_arm/rover_arm.ino
│   ├── launch/rover_drivers.launch  # Main RPi Launch
│   └── scripts/health_monitor.py    # Diagnostics
│
├── rover_control/                   # [PC] AI & NAVIGATION
│   ├── config/                      # Nav Stack Parameters (Costmaps)
│   ├── launch/
│   │   ├── rover_slam.launch        # Mapping
│   │   ├── rover_nav.launch         # Autonomous Driving
│   │   ├── rover_complete.launch    # Full System Start
│   │   └── rover_webui.launch       # Web Server
│   ├── scripts/
│   │   ├── kinematics.py            # Wheel Control Logic
│   │   ├── arm_bridge.py            # MoveIt -> Arduino Bridge
│   │   └── autonomous_mission.py    # Auto-Mission Script
│   └── webui/index.html             # Dashboard Interface
│
├── rover_description/               # [PC] PHYSICAL MODEL
│   └── urdf/rover.urdf              # Robot dimensions & joints
│
└── rover_moveit_config/             # [PC] ARM CONFIG
    └── ... (Generated by Setup Assistant)
```

#

### 5. Next Step
Your system is fully defined. To begin construction:

  1. Flash the Arduinos.
  2. Build the catkin workspace on both machines.
  3. Launch rover_drivers.launch on the Pi.
  4. Launch rover_webui.launch on the PC and open your browser to verify the connection.

#

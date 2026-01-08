
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Bool.h>
#include <AccelStepper.h>

ros::NodeHandle nh;

// --- CONFIGURATION ---
// Shared Enable Pin (Connect to EN on all 5 drivers)
#define COMMON_ENABLE_PIN 32

// Pin Definitions for 5 Axes
// Axis 1 (Shoulder Pan)
#define DIR_1 22
#define STEP_1 23
#define LIM_1 2

// Axis 2 (Shoulder Lift)
#define DIR_2 24
#define STEP_2 25
#define LIM_2 3

// Axis 3 (Elbow)
#define DIR_3 26
#define STEP_3 27
#define LIM_3 4

// Axis 4 (Wrist Pitch)
#define DIR_4 28
#define STEP_4 29
#define LIM_4 5

// Axis 5 (Wrist Roll)
#define DIR_5 30
#define STEP_5 31
#define LIM_5 6

// --- OBJECTS ---
// Interface Type 1 = Stepper Driver (A4988/DRV8825)
AccelStepper axis1(1, STEP_1, DIR_1);
AccelStepper axis2(1, STEP_2, DIR_2);
AccelStepper axis3(1, STEP_3, DIR_3);
AccelStepper axis4(1, STEP_4, DIR_4);
AccelStepper axis5(1, STEP_5, DIR_5);

// Array for easy iteration
AccelStepper* axes[] = {&axis1, &axis2, &axis3, &axis4, &axis5};
const int limit_pins[] = {LIM_1, LIM_2, LIM_3, LIM_4, LIM_5};

// --- CALLBACKS ---

// 1. Move Command: Receives target steps for all 5 axes
void armCallback(const std_msgs::Int16MultiArray& cmd) {
  // Ensure we receive data for all 5 axes
  if (cmd.data_length >= 5) {
    for(int i=0; i<5; i++){
      axes[i]->moveTo(cmd.data[i]);
    }
  }
}

// 2. Enable/Disable Command: Manual power control
// True = Motors ON (Holding Torque), False = Motors OFF (Free Spin)
void enableCallback(const std_msgs::Bool& msg) {
  if(msg.data) {
    digitalWrite(COMMON_ENABLE_PIN, LOW); // LOW is ON for A4988
  } else {
    digitalWrite(COMMON_ENABLE_PIN, HIGH); // HIGH is OFF
  }
}

// 3. Calibration Command: Simple homing
void calibCallback(const std_msgs::Bool& msg) {
  if(msg.data) {
    // Enable motors just in case they were off
    digitalWrite(COMMON_ENABLE_PIN, LOW);
    
    // Simple homing routine (Blocking)
    // Moves constantly until limit switch is triggered
    for(int i=0; i<5; i++){
       // Move backwards slowly
       axes[i]->setSpeed(-200); 
       
       // While limit switch is NOT pressed (assuming LOW when pressed)
       // Change to (digitalRead(...) == HIGH) if your switch is Normally Open and wires to GND without pullup
       while(digitalRead(limit_pins[i]) == HIGH) { 
          axes[i]->runSpeed();
       }
       
       // Stop and reset position
       axes[i]->setCurrentPosition(0);
       axes[i]->setSpeed(0);
       
       // Optional: Move forward slightly to clear the switch
       axes[i]->runToNewPosition(50); 
       axes[i]->setCurrentPosition(0);
    }
  }
}

// --- ROS SUBSCRIBERS ---
ros::Subscriber<std_msgs::Int16MultiArray> sub_arm("rover/arm_cmd", armCallback);
ros::Subscriber<std_msgs::Bool> sub_enable("rover/arm_enable", enableCallback);
ros::Subscriber<std_msgs::Bool> sub_calib("rover/calibrate", calibCallback);

void setup() {
  // 1. Init Serial/ROS
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_arm);
  nh.subscribe(sub_enable);
  nh.subscribe(sub_calib);

  // 2. Init Enable Pin
  pinMode(COMMON_ENABLE_PIN, OUTPUT);
  digitalWrite(COMMON_ENABLE_PIN, LOW); // Default: Motors Enabled (Holding Torque)

  // 3. Init Limit Switches
  for(int i=0; i<5; i++) {
    pinMode(limit_pins[i], INPUT_PULLUP);
  }

  // 4. Init Stepper Parameters
  // Tune these values based on your robot's weight and gear ratio!
  for(int i=0; i<5; i++) {
    axes[i]->setMaxSpeed(1000);      // Max steps per second
    axes[i]->setAcceleration(500);   // Steps per second^2
  }
}

void loop() {
  // Non-blocking run call (Must be called as fast as possible)
  for(int i=0; i<5; i++) {
    axes[i]->run();
  }
  
  nh.spinOnce();
}

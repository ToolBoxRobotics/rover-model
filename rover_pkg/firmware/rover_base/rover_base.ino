
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_INA219.h>
#include "PinChangeInterrupt.h"

// --- CONFIGURATION ---

// 1. PCA9685 Setup (Address 0x40 default)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Motor PWM Channels on PCA9685 (0-5)
const int CH_M_PWM[] = {0, 1, 2, 3, 4, 5}; 

// Servo Channels on PCA9685 (6-9)
const int CH_SERVO[] = {6, 7, 8, 9};

// 2. Arduino Mega GPIO Connections
// Motor Direction Pins (Direct Logic)
const int PIN_M_DIR[] = {22, 23, 24, 25, 26, 27};

// Encoder Pins (A=Interrupt, B=Direction)
const int PIN_ENC_A[] = {A0, A2, A4, A8, A10, A12}; 
const int PIN_ENC_B[] = {A1, A3, A5, A9, A11, A13};
volatile long encoder_ticks[6] = {0,0,0,0,0,0};

// 3. I2C Multiplexer (TCA9548A)
#define TCA_ADDR 0x70
#define MPU_PORT 0
#define INA_PORT 1

// --- OBJECTS ---
ros::NodeHandle nh;
Adafruit_MPU6050 mpu;
Adafruit_INA219 ina219;

// --- MESSAGES ---
sensor_msgs::Imu imu_msg;
sensor_msgs::BatteryState bat_msg;
std_msgs::Float32MultiArray enc_msg;

// --- HELPER FUNCTIONS ---

void tcaSelect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

// Map servo angle (0-180) to PCA9685 Pulse (approx 150-600)
// Calibrate these min/max values for your specific servos!
int angleToPulse(int angle) {
  int pulse = map(angle, 0, 180, 150, 600);
  return constrain(pulse, 150, 600);
}

// Encoder ISRs
void readEnc0() { if(digitalRead(PIN_ENC_B[0])) encoder_ticks[0]++; else encoder_ticks[0]--; }
void readEnc1() { if(digitalRead(PIN_ENC_B[1])) encoder_ticks[1]++; else encoder_ticks[1]--; }
void readEnc2() { if(digitalRead(PIN_ENC_B[2])) encoder_ticks[2]++; else encoder_ticks[2]--; }
void readEnc3() { if(digitalRead(PIN_ENC_B[3])) encoder_ticks[3]++; else encoder_ticks[3]--; }
void readEnc4() { if(digitalRead(PIN_ENC_B[4])) encoder_ticks[4]++; else encoder_ticks[4]--; }
void readEnc5() { if(digitalRead(PIN_ENC_B[5])) encoder_ticks[5]++; else encoder_ticks[5]--; }

// --- CALLBACKS ---

// Expected Input: [Speed_M1...M6 (0-255), Angle_S1...S4 (0-180)]
void driveCallback(const std_msgs::Float32MultiArray& cmd) {
  if (cmd.data_length < 10) return;

  // 1. Update Motors
  for(int i=0; i<6; i++){
    int speed_in = (int)cmd.data[i]; // -255 to 255
    
    // Direction Logic (Mega GPIO)
    digitalWrite(PIN_M_DIR[i], speed_in > 0 ? HIGH : LOW);
    
    // Speed Logic (PCA9685 12-bit PWM: 0-4095)
    int pwm_val = map(abs(speed_in), 0, 255, 0, 4095);
    pwm.setPWM(CH_M_PWM[i], 0, pwm_val);
  }

  // 2. Update Servos
  for(int i=0; i<4; i++){
    int angle = (int)cmd.data[6+i];
    pwm.setPWM(CH_SERVO[i], 0, angleToPulse(angle));
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub_drive("rover/drive_cmd", driveCallback);
ros::Publisher pub_enc("rover/encoders", &enc_msg);
ros::Publisher pub_imu("rover/imu", &imu_msg);
ros::Publisher pub_bat("rover/battery", &bat_msg);

void setup() {
  // ROS Init
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub_drive);
  nh.advertise(pub_enc);
  nh.advertise(pub_imu);
  nh.advertise(pub_bat);

  Wire.begin();

  // 1. Init PCA9685 (Main Bus)
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50); // 50Hz is standard for Servos and good for generic DC motors
  
  // 2. Init Digital Pins (Direction)
  for(int i=0; i<6; i++) {
    pinMode(PIN_M_DIR[i], OUTPUT);
    digitalWrite(PIN_M_DIR[i], LOW);
  }

  // 3. Init Encoders
  for(int i=0; i<6; i++) {
    pinMode(PIN_ENC_A[i], INPUT_PULLUP);
    pinMode(PIN_ENC_B[i], INPUT_PULLUP);
  }
  attachPCINT(digitalPinToPCINT(PIN_ENC_A[0]), readEnc0, RISING);
  attachPCINT(digitalPinToPCINT(PIN_ENC_A[1]), readEnc1, RISING);
  attachPCINT(digitalPinToPCINT(PIN_ENC_A[2]), readEnc2, RISING);
  attachPCINT(digitalPinToPCINT(PIN_ENC_A[3]), readEnc3, RISING);
  attachPCINT(digitalPinToPCINT(PIN_ENC_A[4]), readEnc4, RISING);
  attachPCINT(digitalPinToPCINT(PIN_ENC_A[5]), readEnc5, RISING);

  // 4. Init Sensors (Behind TCA9548A)
  
  // MPU6050 Setup
  tcaSelect(MPU_PORT);
  if(!mpu.begin()) { 
    // Handle error (blink LED?) 
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // INA219 Setup
  tcaSelect(INA_PORT);
  ina219.begin();

  // Allocate memory for encoder message
  enc_msg.data_length = 6;
  enc_msg.data = (float*)malloc(sizeof(float) * 6);
}

void loop() {
  static unsigned long prev_time = 0;
  
  // 20Hz Publish Rate (Every 50ms)
  if(millis() - prev_time > 50) { 
    prev_time = millis();

    // 1. Publish Encoders
    for(int i=0; i<6; i++) enc_msg.data[i] = (float)encoder_ticks[i];
    pub_enc.publish(&enc_msg);

    // 2. Publish IMU
    tcaSelect(MPU_PORT);
    sensors_event_t a, g, temp;
    if(mpu.getEvent(&a, &g, &temp)) {
        imu_msg.header.frame_id = "imu_link";
        imu_msg.header.stamp = nh.now();
        imu_msg.linear_acceleration.x = a.acceleration.x;
        imu_msg.linear_acceleration.y = a.acceleration.y;
        imu_msg.linear_acceleration.z = a.acceleration.z;
        imu_msg.angular_velocity.x = g.gyro.x;
        imu_msg.angular_velocity.y = g.gyro.y;
        imu_msg.angular_velocity.z = g.gyro.z;
        pub_imu.publish(&imu_msg);
    }

    // 3. Publish Battery
    tcaSelect(INA_PORT);
    // Note: Re-init or check availability might be needed if INA drops out
    bat_msg.voltage = ina219.getBusVoltage_V();
    bat_msg.current = ina219.getCurrent_mA() / 1000.0; // Convert to Amps
    pub_bat.publish(&bat_msg);
  }

  nh.spinOnce();
}

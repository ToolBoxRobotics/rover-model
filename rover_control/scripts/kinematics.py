#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math

# Rover Dimensions
L = 0.5 # Length between front and rear
W = 0.4 # Width

def cmd_vel_cb(msg):
    v = msg.linear.x
    omega = msg.angular.z
    
    # Calculate steering angles and wheel speeds here (Ackerman or Skid)
    # This is a simplified example assuming skid steer helper for 6 wheels
    # Real implementation needs Ackerman math for the corner servos
    
    left_speed = v - (omega * W / 2)
    right_speed = v + (omega * W / 2)
    
    # Map m/s to PWM (approximate)
    pwm_l = int(left_speed * 255)
    pwm_r = int(right_speed * 255)
    
    # Prepare message: [6x Speeds, 4x Servo Angles]
    # Servos set to 90 (straight) for skid steer mode
    data = [pwm_l, pwm_r, pwm_l, pwm_r, pwm_l, pwm_r, 90, 90, 90, 90]
    
    pub_msg = Float32MultiArray()
    pub_msg.data = data
    pub.publish(pub_msg)

rospy.init_node('rover_kinematics')
pub = rospy.Publisher('rover/drive_cmd', Float32MultiArray, queue_size=10)
rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)
rospy.spin()

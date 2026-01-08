
#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray

# Configuration: Steps per revolution for your Stepper + Gearbox
# Example: 200 steps/rev * 16 microsteps * 5:1 gearbox = 16000 steps/rev
STEPS_PER_REV = [16000, 16000, 16000, 8000, 3200] 

class ArmBridge:
    def __init__(self):
        rospy.init_node('arm_bridge')
        
        # Subscribe to the topic MoveIt publishes to (usually /joint_states or /move_group/fake_controller_joint_states)
        self.sub = rospy.Subscriber('/move_group/fake_controller_joint_states', JointState, self.cb)
        
        # Publisher to Arduino
        self.pub = rospy.Publisher('rover/arm_cmd', Int16MultiArray, queue_size=10)
        
        self.current_steps = [0, 0, 0, 0, 0]

    def cb(self, msg):
        # Map ROS joint names to our indices 0-4
        # Note: msg.name gives joint names, msg.position gives angles in radians
        
        cmd_steps = [0] * 5
        
        # Simple mapping (You must ensure URDF joint names match these strings)
        joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5']
        
        for i, joint_name in enumerate(joints):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                angle_rad = msg.position[idx]
                
                # Convert Rad -> Steps
                # Steps = (Angle / 2Pi) * StepsPerRev
                steps = int((angle_rad / (2 * math.pi)) * STEPS_PER_REV[i])
                cmd_steps[i] = steps

        # Publish to Arduino
        out_msg = Int16MultiArray()
        out_msg.data = cmd_steps
        self.pub.publish(out_msg)

if __name__ == '__main__':
    ArmBridge()
    rospy.spin()

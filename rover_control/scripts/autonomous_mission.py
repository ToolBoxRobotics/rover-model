#!/usr/bin/env python3

import rospy
import actionlib
import sys
import copy
import math

# Navigation Imports
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion

# MoveIt Imports (Arm Control)
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class AutonomousRover:
    def __init__(self):
        # 1. Initialize Nodes and Commanders
        rospy.init_node('autonomous_mission_node', anonymous=True)
        
        # Initialize MoveIt (Arm)
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "arm" # Must match what you named it in Setup Assistant
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        # Initialize Navigation Client
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.nav_client.wait_for_server()
        rospy.loginfo("Connected to move_base and MoveIt!")

    # --- NAVIGATION FUNCTION ---
    def go_to_waypoint(self, x, y, yaw):
        rospy.loginfo(f"Navigating to: x={x}, y={y}")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        
        # Convert Yaw (degrees) to Quaternion
        # (Simplified conversion for 2D floor navigation)
        rad = yaw * (math.pi / 180.0)
        goal.target_pose.pose.orientation.z = math.sin(rad / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(rad / 2.0)

        self.nav_client.send_goal(goal)
        wait = self.nav_client.wait_for_result()
        
        if not wait:
            rospy.logerr("Action server not available!")
            return False
        else:
            return self.nav_client.get_result()

    # --- ARM HELPER FUNCTIONS ---
    def arm_go_to_named(self, name):
        # 'home', 'carry', 'ground' must be defined in your MoveIt Setup
        rospy.loginfo(f"Moving arm to pose: {name}")
        self.move_group.set_named_target(name)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def execute_digging_motion(self):
        rospy.loginfo("Computing Digging Cartesian Path...")
        
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        
        # Step 1: Lower straight down 5cm
        wpose.position.z -= 0.05 
        waypoints.append(copy.deepcopy(wpose))

        # Step 2: Scoop (Forward 5cm and Up 2cm)
        wpose.position.x += 0.05
        wpose.position.z += 0.02
        waypoints.append(copy.deepcopy(wpose))
        
        # Step 3: Lift Up 10cm (Retract)
        wpose.position.z += 0.10
        waypoints.append(copy.deepcopy(wpose))

        # Compute Path (Resolution 1cm)
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0          # jump_threshold
        )
        
        if fraction < 0.9:
            rospy.logwarn(f"Path planning incomplete! Only {fraction*100}% planned.")
            return False

        rospy.loginfo("Executing Dig...")
        self.move_group.execute(plan, wait=True)
        return True

    # --- MAIN MISSION ---
    def run_mission(self):
        # 1. Start: Ensure Arm is in safe Carry Mode
        self.arm_go_to_named("home")
        
        # 2. Navigate to Sample Site (e.g., x=2.0 meters, y=0.0 meters)
        # You get these coords by clicking "Publish Point" in Rviz
        rospy.loginfo("--- Phase 1: Transit ---")
        self.go_to_waypoint(2.0, 0.0, 0)
        
        # 3. Deploy Arm to Ground
        rospy.loginfo("--- Phase 2: Deployment ---")
        # Ensure you saved a pose named 'ready_to_dig' in MoveIt setup assistant
        # that places the tip near the ground
        self.arm_go_to_named("ready_to_dig")
        
        # 4. Dig
        rospy.loginfo("--- Phase 3: Extraction ---")
        self.execute_digging_motion()
        
        # 5. Pack up
        rospy.loginfo("--- Phase 4: Stowing ---")
        self.arm_go_to_named("home")
        
        # 6. Return to Base (x=0, y=0)
        rospy.loginfo("--- Phase 5: Return ---")
        self.go_to_waypoint(0.0, 0.0, 180)
        
        rospy.loginfo("MISSION COMPLETE")

if __name__ == '__main__':
    try:
        rover = AutonomousRover()
        rover.run_mission()
    except rospy.ROSInterruptException:
        pass

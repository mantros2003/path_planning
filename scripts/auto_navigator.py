#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

class TrialOrchestrator:
    def __init__(self):
        rospy.init_node('trial_orchestrator', anonymous=True)

        # Define coordinates
        self.start_pose = self.create_pose(x=4.0, y=-4.0, theta_w=1.0)
        self.target_pose = self.create_pose(x=-4.0, y=4.0, theta_w=1.0)
        
        # Define how many trips to make
        self.total_trials = rospy.get_param('~trials', 10)
        self.robot_name = rospy.get_param("~robot_name", "mobile_bot")

        # Connect to the navigation action server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for navigation action server to come online...")
        self.client.wait_for_server()

        # Connect to Gazebo's model state service
        rospy.loginfo("Waiting for GAzebo's /gazebo/set_model_state service")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

        rospy.loginfo("Connected. Ready to begin trials.")

    def create_pose(self, x, y, theta_w):
        """Helper to cleanly build a Pose object."""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        # Z-axis rotation (quaternion) is required for differential drive robots
        pose.orientation.w = theta_w 
        return pose

    def send_navigation_command(self, pose, destination_name):
        """Sends a goal and blocks until the robot finishes moving."""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        rospy.loginfo(f"Sending command to navigate to: {destination_name}")
        self.client.send_goal(goal)

        # Pause script execution until the navigation stack reports completion
        self.client.wait_for_result()
        
        # Check the outcome (3 = SUCCEEDED)
        state = self.client.get_state()
        if state == 3:
            rospy.loginfo(f"Success! Reached {destination_name}.")
            return True
        else:
            rospy.logwarn(f"Failed to reach {destination_name}. State code: {state}")
            return False

    def teleport_to_start(self):
        rospy.loginfo(f"Teleporting {self.robot_name} to the start.")

        state_msg = ModelState()
        state_msg.model_name = self.robot_name
        state_msg.pose = self.start_pose

        state_msg.twist.linear.x = 0.0
        state_msg.twist.linear.y = 0.0
        state_msg.twist.angular.z = 0.0

        state_msg.reference_frame = "world"

        try:
            resp = self.set_model_state(state_msg)
            if resp.success:
                rospy.loginfo("Teleport successful")
            else:
                rospy.loginfo("Failed to teleport")
        except rospy.ServiceException as e:
            rospy.logerr(f"Gazebo service call failed: {e}")

    def run_trials(self):
        """The main loop that drives the robot back and forth."""
        for i in range(1, self.total_trials + 1):
            rospy.loginfo("\n" + "="*40)
            rospy.loginfo(f" INITIATING BENCHMARK TRIAL {i} OF {self.total_trials}")
            rospy.loginfo("="*40)

            # Drive to the target across the dynamic obstacles
            self.send_navigation_command(self.target_pose, "Target Destination")
            
            # Brief pause to let local obstacle maps clear and the logger save the CSV
            rospy.sleep(2.0)

            # Drive back to the starting point to reset for the next trial
            self.teleport_to_start()
            
            # Brief pause before launching the next trial
            rospy.sleep(3.0)

        rospy.loginfo("All automated trials completed successfully.")

if __name__ == '__main__':
    try:
        orchestrator = TrialOrchestrator()
        orchestrator.run_trials()
    except rospy.ROSInterruptException:
        pass

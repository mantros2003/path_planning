#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose

class TrialOrchestrator:
    def __init__(self):
        rospy.init_node('trial_orchestrator', anonymous=True)

        # Define coordinates
        self.start_pose = self.create_pose(x=4.0, y=-4.0, theta_w=1.0)
        self.target_pose = self.create_pose(x=-4.0, y=4.0, theta_w=1.0)
        
        # Define how many trips to make
        self.total_trials = rospy.get_param('~trials', 10)

        # Connect to the navigation action server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for navigation action server to come online...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to navigation server. Ready to begin trials.")

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

    def run_trials(self):
        """The main loop that drives the robot back and forth."""
        for i in range(1, self.total_trials + 1):
            rospy.loginfo("\n" + "="*40)
            rospy.loginfo(f" INITIATING BENCHMARK TRIAL {i} OF {self.total_trials}")
            rospy.loginfo("="*40)

            # Phase 1: Drive to the target across the dynamic obstacles
            self.send_navigation_command(self.target_pose, "Target Destination")
            
            # Brief pause to let local obstacle maps clear and the logger save the CSV
            rospy.sleep(2.0)

            # Phase 2: Drive back to the starting point to reset for the next trial
            rospy.loginfo("Returning to starting position to reset...")
            self.send_navigation_command(self.start_pose, "Home Position")
            
            # Brief pause before launching the next trial
            rospy.sleep(2.0)

        rospy.loginfo("All automated trials completed successfully.")

if __name__ == '__main__':
    try:
        orchestrator = TrialOrchestrator()
        orchestrator.run_trials()
    except rospy.ROSInterruptException:
        pass

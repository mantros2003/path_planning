#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Path

def path_callback(msg):
    """
    Callback function that gets executed every time a new path is published.
    """
    path_length = len(msg.poses)
    rospy.loginfo(f"Received a new global path with {path_length} poses.")

    # Example: Iterate through the first 3 poses in the path
    if path_length > 0:
        rospy.loginfo("First few points of the path:")
        for i in range(min(3, path_length)):
            pose = msg.poses[i].pose
            x = pose.position.x
            y = pose.position.y
            rospy.loginfo(f"  Point {i}: x={x:.2f}, y={y:.2f}")

def global_path_listener():
    # Initialize the ROS node
    rospy.init_node('global_path_reader', anonymous=True)

    # Define the topic name based on your planner configuration
    # Note: Change 'GlobalPlanner' to 'NavfnROS' if that is what your system uses.
    topic_name = '/move_base/DWAPlannerROS/global_plan'

    # Subscribe to the topic
    rospy.Subscriber(topic_name, Path, path_callback)
    
    rospy.loginfo(f"Subscribed to {topic_name}. Waiting for paths...")

    # Keep Python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        global_path_listener()
    except rospy.ROSInterruptException:
        pass

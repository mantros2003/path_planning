#!/usr/bin/env python3

import rospy
import math
import csv
import os
from nav_msgs.msg import Path

class PathMetricsLogger:
    def __init__(self):
        rospy.init_node('path_metrics_logger', anonymous=True)

        # Parameters
        self.path_topic = rospy.get_param('~path_topic', '/move_base/GlobalPlanner/plan')
        self.csv_file = rospy.get_param('~csv_path', '/tmp/rrtx_path_metrics.csv')
        
        # State variable
        self.last_path_time = None

        # Setup CSV
        self.init_csv()

        # Subscriber
        rospy.Subscriber(self.path_topic, Path, self.path_callback)

        rospy.loginfo(f"Listening to {self.path_topic}. Saving data to: {self.csv_file}")

    def init_csv(self):
        """Creates the CSV file and writes headers if it doesn't exist."""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(["ROS_Time_sec", "Replanning_Latency_sec", "Path_Length_m"])

    def path_callback(self, msg):
        # 1. Calculate Planning Latency
        current_time = msg.header.stamp.to_sec()
        latency = 0.0
        
        if self.last_path_time is not None:
            latency = current_time - self.last_path_time
            
            # Reset if it's a new goal (not a continuous replan)
            if latency > 5.0:
                latency = 0.0 
                
        self.last_path_time = current_time

        # 2. Calculate Path Length
        path_length = 0.0
        poses = msg.poses
        
        if len(poses) > 1:
            for i in range(1, len(poses)):
                x1 = poses[i-1].pose.position.x
                y1 = poses[i-1].pose.position.y
                x2 = poses[i].pose.position.x
                y2 = poses[i].pose.position.y
                
                path_length += math.hypot(x2 - x1, y2 - y1)

        # 3. Save to CSV
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                round(current_time, 4), 
                round(latency, 4), 
                round(path_length, 4)
            ])

if __name__ == '__main__':
    try:
        PathMetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

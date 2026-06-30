#!/usr/bin/env python3

import rospy
import math
import csv
import os
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray

class AutomatedMetricsLogger:
    def __init__(self):
        rospy.init_node('automated_metrics_logger', anonymous=True)

        # Configurable Parameters
        self.path_topic = rospy.get_param('~path_topic', '/move_base/GlobalPlanner/plan')
        self.status_topic = rospy.get_param('~status_topic', '/move_base/status')
        self.csv_file = rospy.get_param('~csv_path', '/tmp/rrtx_automated_metrics.csv')
        
        # State Variables
        self.is_recording = False
        self.trial_id = 0
        
        # Trial Metrics
        self.last_path_time = None
        self.latencies = []
        self.latest_path_length = 0.0

        # Setup CSV and resume Trial ID
        self.setup_csv_and_resume()

        # Subscribers
        rospy.Subscriber(self.status_topic, GoalStatusArray, self.status_callback)
        rospy.Subscriber(self.path_topic, Path, self.path_callback)

        rospy.loginfo("Automated Logger Ready. Waiting for a navigation goal...")

    def setup_csv_and_resume(self):
        """Creates CSV if missing, or reads the last Trial_ID to resume safely."""
        if not os.path.exists(self.csv_file):
            # File doesn't exist, create it and write headers
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Trial_ID", 
                    "Outcome", 
                    "Avg_Replanning_Latency_sec", 
                    "Max_Latency_sec", 
                    "Final_Path_Length_m", 
                    "Total_Replans"
                ])
            self.trial_id = 0
            rospy.loginfo(f"Created new CSV at: {self.csv_file}")
        else:
            # File exists, scan it to find the last Trial_ID
            last_id = 0
            try:
                with open(self.csv_file, mode='r') as file:
                    reader = csv.reader(file)
                    next(reader, None)  # Skip the header row
                    for row in reader:
                        # Ensure the row isn't empty and the first column is a number
                        if row and row[0].isdigit():
                            last_id = int(row[0])
                
                self.trial_id = last_id
                rospy.loginfo(f"Found existing CSV. Resuming from Trial {self.trial_id + 1}")
            except Exception as e:
                rospy.logerr(f"Failed to read existing CSV file: {e}")
                rospy.logwarn("Defaulting Trial ID to 0. Watch out for data overwrites!")

    def status_callback(self, msg):
        """State machine to start and stop recording based on move_base status."""
        if not msg.status_list:
            return

        # Extract the latest status code
        latest_status = msg.status_list[-1].status

        # Trigger: A new goal was received, robot is moving
        if latest_status == 1 and not self.is_recording:
            self.is_recording = True
            self.trial_id += 1
            
            # Reset metrics for the new trial
            self.latencies = []
            self.latest_path_length = 0.0
            self.last_path_time = None
            
            rospy.loginfo(f"\n--- Trial {self.trial_id} Started ---")

        # Trigger: The robot reached the goal (3) or failed/aborted (4)
        elif latest_status in [3, 4] and self.is_recording:
            self.is_recording = False
            outcome = "SUCCEEDED" if latest_status == 3 else "ABORTED"
            rospy.loginfo(f"--- Trial {self.trial_id} {outcome} ---")
            
            self.save_trial_data(outcome)

    def path_callback(self, msg):
        """Aggregates latency and length ONLY when a trial is active."""
        if not self.is_recording:
            return

        # 1. Calculate Replanning Latency
        current_time = msg.header.stamp.to_sec()
        if self.last_path_time is not None:
            latency = current_time - self.last_path_time
            # Ignore massive time jumps
            if latency < 5.0: 
                self.latencies.append(latency)
        self.last_path_time = current_time

        # 2. Calculate Current Path Length
        poses = msg.poses
        length = 0.0
        if len(poses) > 1:
            for i in range(1, len(poses)):
                x1 = poses[i-1].pose.position.x
                y1 = poses[i-1].pose.position.y
                x2 = poses[i].pose.position.x
                y2 = poses[i].pose.position.y
                
                length += math.hypot(x2 - x1, y2 - y1)
        
        self.latest_path_length = length

    def save_trial_data(self, outcome):
        """Calculates final statistics and appends one row to the CSV."""
        avg_latency = sum(self.latencies) / len(self.latencies) if self.latencies else 0.0
        max_latency = max(self.latencies) if self.latencies else 0.0
        total_replans = len(self.latencies)

        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.trial_id,
                outcome,
                round(avg_latency, 4),
                round(max_latency, 4),
                round(self.latest_path_length, 4),
                total_replans
            ])
            
        rospy.loginfo(f"Trial {self.trial_id} data successfully saved to CSV.")

if __name__ == '__main__':
    try:
        AutomatedMetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

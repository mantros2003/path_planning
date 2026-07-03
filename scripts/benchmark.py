#!/usr/bin/env python3

import rospy
import math
import csv
import os
from nav_msgs.msg import Path
from actionlib_msgs.msg import GoalStatusArray

class ContinuousMetricsLogger:
    def __init__(self):
        rospy.init_node('continuous_metrics_logger', anonymous=True)

        # Configurable Parameters
        self.path_topic = rospy.get_param('~path_topic', '/move_base/DWAPlannerROS/global_plan')
        self.status_topic = rospy.get_param('~status_topic', '/move_base/status')
        self.csv_file = rospy.get_param('~csv_path', '/home/swapnil/rrtx_continuous_metrics.csv')
        
        # State Variables
        self.is_recording = False
        self.trial_id = 0
        
        # Continuous Data Buffer
        self.last_path_time = None
        self.trial_data = [] # Stores tuples of (ros_time, latency, path_length)

        # Setup CSV and resume Trial ID
        self.setup_csv_and_resume()

        # Subscribers
        rospy.Subscriber(self.status_topic, GoalStatusArray, self.status_callback)
        rospy.Subscriber(self.path_topic, Path, self.path_callback)

        rospy.loginfo("Continuous Logger Ready. Waiting for a navigation goal...")

    def setup_csv_and_resume(self):
        """Creates CSV if missing, or reads the last Trial_ID to resume safely."""
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Trial_ID", 
                    "ROS_Time_sec", 
                    "Replanning_Latency_sec", 
                    "Path_Length_m", 
                    "Outcome"
                ])
            self.trial_id = 0
            rospy.loginfo(f"Created new CSV at: {self.csv_file}")
        else:
            last_id = 0
            try:
                with open(self.csv_file, mode='r') as file:
                    reader = csv.reader(file)
                    next(reader, None)  # Skip the header row
                    for row in reader:
                        if row and row[0].isdigit():
                            last_id = int(row[0])
                
                self.trial_id = last_id
                rospy.loginfo(f"Found existing CSV. Resuming from Trial {self.trial_id + 1}")
            except Exception as e:
                rospy.logerr(f"Failed to read existing CSV file: {e}")
                rospy.logwarn("Defaulting Trial ID to 0.")

    def status_callback(self, msg):
        """State machine to start and stop recording based on status."""
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status

        # Trigger: A new goal was received, robot is moving
        if latest_status == 1 and not self.is_recording:
            self.is_recording = True
            self.trial_id += 1
            
            # Reset buffer and timer for the new trial
            self.trial_data = []
            self.last_path_time = None
            
            rospy.loginfo(f"\n--- Trial {self.trial_id} Started (Continuous Logging) ---")

        # Trigger: The robot reached the goal (3) or failed/aborted (4)
        elif latest_status in [3, 4] and self.is_recording:
            self.is_recording = False
            
            # 1 for success, -1 for failure
            final_outcome = 1 if latest_status == 3 else -1
            status_str = "SUCCEEDED" if latest_status == 3 else "ABORTED"
            rospy.loginfo(f"--- Trial {self.trial_id} {status_str}. Writing data... ---")
            
            self.save_trial_data(final_outcome)

    def path_callback(self, msg):
        """Calculates latency and length, and buffers it into memory."""
        if not self.is_recording:
            return

        # 1. Calculate Replanning Latency
        current_time = msg.header.stamp.to_sec()
        latency = 0.0
        
        if self.last_path_time is not None:
            latency = current_time - self.last_path_time
            # Ignore massive time jumps (e.g., simulator pauses)
            if latency > 5.0: 
                latency = 0.0
                
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
        
        # Append to the continuous buffer
        self.trial_data.append((current_time, latency, length))

    def save_trial_data(self, final_outcome_code):
        """Writes the buffered continuous data to the CSV file."""
        if not self.trial_data:
            rospy.logwarn(f"Trial {self.trial_id} completed, but no path data was collected.")
            return

        total_entries = len(self.trial_data)
        
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            
            for index, data_point in enumerate(self.trial_data):
                ros_time = data_point[0]
                latency = data_point[1]
                path_length = data_point[2]
                
                # Determine outcome code (0 for all, except the very last entry)
                is_last_entry = (index == total_entries - 1)
                outcome = final_outcome_code if is_last_entry else 0
                
                writer.writerow([
                    self.trial_id,
                    round(ros_time, 4),
                    round(latency, 4),
                    round(path_length, 4),
                    outcome
                ])
            
        rospy.loginfo(f"Trial {self.trial_id}: Wrote {total_entries} continuous records to CSV.")

if __name__ == '__main__':
    try:
        ContinuousMetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

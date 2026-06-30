#!/bin/python3

import rospy
import csv
import os
import math

from nav_msgs.msg import Odometry
from nav_msgs.msg import Float64
from actionlib_msgs.msg import GoalStatusArray

class BenchmarkLogger:
    def __init__(self):
        rospy.init_node("rrtx_benchmark_logger", anonymous=true)

        self.csv_file = rospy.get_param("~csv_path", "/tmp/rrtx_benchmark_results.csv")
        self.csv_init()

        # State variables
        self.is_mission_active = False
        self.mission_status = "UNKNOWN"
        self.last_pose = None

        # Metrics to track
        self.total_distance = 0.0
        self.latency_record = []

        # Subcriptions
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/rrtx/planning_latency", Float64, self.latency_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)

    def csv_init(self):
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    "Status",
                    "Latency",
                    "Replans"
                ])

    def latency_callback(self, msg):
        if status.is_mission_active:
            self.latency_records.append(msg.data)

    def status_callback(self, msg):
        if not msg.status_list:
            return

        latest_status = msg.status_list[-1].status

        if latest_status == 1 and not self.is_mission_active:
            self.reset_metrics()
            self.is_mission_active = True
            rospy.loginfo("[Bencmark Logger] Started tracking metrics")

        elif latest_status in [3, 4] and self.is_mission_active:
            self.is_mission_active = False
            self.mission_status = "SUCCEEDED" if latest_status == 3 else "ABORTED"
            self.save_trial_result()
            rospy.loginfo(f"Mission {self.mission_status}. Saving data")

    def reset_metrics(self):
        self.total_distance = 0.0
        self.latency_records = []
        self.last_pose = None
        self.mission_status = "UNKNOWN"

    def save_trial(self):
        with open(self.csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                self.mission_status,
                round(self.total_distance, 4),
                round(mean_latency, 4),
                total_replans
            ])

#!/usr/bin/env python

import rospy
import random
import time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32


class FaultInjection:
    def __init__(self):
        rospy.init_node("fault_injection_scan")

        # Get fault type from ROS parameter
        self.fault_type = rospy.get_param("~fault_type", "none")  # Default: no fault

        # Publisher for faulty scan topic
        self.faulty_pub = rospy.Publisher("/faulty_scan", LaserScan, queue_size=10)

        # Subscribe to the original scan topic
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        rospy.loginfo(f"Fault Injection Node Started - Fault Type: {self.fault_type}")

        self.current_marker = None
        self.marker_sub = rospy.Subscriber("/current_marker_goal", Int32, self.marker_callback)

    def marker_callback(self, msg):
        self.current_marker = msg.data

    def scan_callback(self, msg):
        faulty_scan = msg  # Copy the original scan message

        if self.fault_type == "random_noise" and self.current_marker == 4:
            self.inject_random_noise(faulty_scan)
        elif self.fault_type == "data_dropout" and self.current_marker == 4:
            self.inject_data_dropout(faulty_scan)
        elif self.fault_type == "stuck_at" and self.current_marker == 4:
            self.inject_stuck_at(faulty_scan)
        elif self.fault_type == "range_scaling" and self.current_marker == 4:
            self.inject_range_scaling(faulty_scan)
        elif self.fault_type == "time_delay" and self.current_marker == 4:
            self.inject_time_delay(faulty_scan)
        elif self.fault_type == "complete_failure" and self.current_marker == 4:
            self.inject_complete_failure()
            return  # Do not publish anything

        # Publish the modified scan data
        self.faulty_pub.publish(faulty_scan)

    # Fault Injection Methods
    def inject_random_noise(self, scan):
        noise_level = 1.0  # Adjust noise level
        scan.ranges = [
            r + random.uniform(-noise_level, noise_level) if r > 0 else r
            for r in scan.ranges
        ]
        rospy.loginfo("Injected Random Noise")

    def inject_data_dropout(self, scan):
        dropout_rate = 0.3  # 30% data loss
        scan.ranges = [
            r if random.random() > dropout_rate else float("inf")
            for r in scan.ranges
        ]
        rospy.loginfo("Injected Data Dropout")

    def inject_stuck_at(self, scan):
        stuck_value = 2.0  # Fixed value for stuck sensor
        scan.ranges = [stuck_value] * len(scan.ranges)
        rospy.loginfo("Injected Stuck-at Fault")

    def inject_range_scaling(self, scan):
        scale_factor = 1.5  # Example: Overestimating distances
        scan.ranges = [r * scale_factor for r in scan.ranges]
        rospy.loginfo("Injected Range Scaling")

    def inject_time_delay(self, scan):
        delay_time = 0.5  # 500ms delay
        rospy.sleep(delay_time)
        rospy.loginfo("Injected Time Delay")

    def inject_complete_failure(self):
        rospy.logwarn("Injected Complete Failure - No scan data published")

if __name__ == "__main__":
    FaultInjection()
    rospy.spin()

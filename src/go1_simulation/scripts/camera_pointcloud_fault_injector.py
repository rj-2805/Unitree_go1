#!/usr/bin/env python

import rospy
import sys
import random
import time
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

class PointCloudFaultInjector:
    def __init__(self, fault_type):
        rospy.init_node('fault_injector', anonymous=True)

        # Subscribe to the original point cloud topic
        self.sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.callback)

        # Publish the faulty point cloud
        self.pub = rospy.Publisher('/camera/depth/color/faulty_points', PointCloud2, queue_size=1)

        # Fault type selection
        self.fault_type = fault_type

        # Fault parameters
        self.noise_stddev = 0.05  # Standard deviation for noise injection
        self.corruption_rate = 0.2  # Percentage of points to corrupt
        self.dropout_probability = 0.1  # Probability of skipping a message
        self.delay_time = 0.5  # Delay in seconds for latency simulation
        self.frame_error_probability = 0.1  # Probability of modifying frame_id

        rospy.loginfo(f"Fault Injector initialized with fault type: {self.fault_type}")

    def callback(self, msg):
        faulty_header = msg.header  # Copy original header

        # **1. Sensor Dropout (Stop publishing)**
        if self.fault_type == "dropout" and random.random() < self.dropout_probability:
            rospy.logwarn("Fault Injection: Sensor Dropout - Skipping message")
            return

        # **2. Latency Simulation (Artificial Delay)**
        if self.fault_type == "delay":
            rospy.logwarn(f"Fault Injection: Introducing {self.delay_time} sec delay")
            time.sleep(self.delay_time)

        # Extract field names dynamically
        field_names = [field.name for field in msg.fields]

        # Convert point cloud to list
        points = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
        faulty_points = []

        for p in points:
            p = list(p)  # Convert tuple to list for modification
            
            # **3. Noise Injection (Add Gaussian Noise)**
            if self.fault_type == "noise":
                p[0] += random.gauss(0, self.noise_stddev)  # X
                p[1] += random.gauss(0, self.noise_stddev)  # Y
                p[2] += random.gauss(0, self.noise_stddev)  # Z

            # **4. Data Corruption (Modify or Delete Points)**
            if self.fault_type == "corrupt" and random.random() < self.corruption_rate:
                p[0], p[1], p[2] = float('nan'), float('nan'), float('nan')  # Corrupt by setting NaN

            faulty_points.append(tuple(p))  # Convert back to tuple

        # **5. Frame Transformation Errors**
        if self.fault_type == "tf_error" and random.random() < self.frame_error_probability:
            faulty_header.frame_id = "wrong_frame"  # Modify TF frame ID
            rospy.logwarn("Fault Injection: TF Error - Modified frame_id to 'wrong_frame'")

        # Create and publish the faulty point cloud
        faulty_cloud = pc2.create_cloud(faulty_header, msg.fields, faulty_points)
        self.pub.publish(faulty_cloud)

        rospy.loginfo("Fault Injection: Published faulty point cloud")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: rosrun your_package fault_injector.py <fault_type>")
        print("Fault types: noise, corrupt, delay, dropout, tf_error")
        sys.exit(1)

    fault_type = sys.argv[1].lower()
    valid_faults = ["noise", "corrupt", "delay", "dropout", "tf_error"]

    if fault_type not in valid_faults:
        print(f"Invalid fault type '{fault_type}'. Choose from: {', '.join(valid_faults)}")
        sys.exit(1)

    try:
        injector = PointCloudFaultInjector(fault_type)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

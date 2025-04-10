#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class DepthFaultInjector:
    def __init__(self, fault_type):
        rospy.init_node('depth_fault_injector', anonymous=True)
        self.bridge = CvBridge()
        self.fault_type = fault_type

        # Subscribe to the original depth image
        self.sub = rospy.Subscriber("/camera/depth/image_raw", Image, self.callback)

        # Publish the faulty depth image
        self.pub = rospy.Publisher("/camera/depth/faulty_image", Image, queue_size=10)

    def callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if self.fault_type == "blur":
                # Apply Gaussian Blur
                faulty_depth_image = cv2.GaussianBlur(depth_image, (5, 5), 2)

            elif self.fault_type == "noise":
                # Inject Gaussian noise
                noise = np.random.normal(0, 50, depth_image.shape).astype(np.float32)
                faulty_depth_image = np.clip(depth_image + noise, 0, 65535)  # Ensure valid depth range

            elif self.fault_type == "both":
                # Apply both blur and noise
                blurred_image = cv2.GaussianBlur(depth_image, (5, 5), 2)
                noise = np.random.normal(0, 50, blurred_image.shape).astype(np.float32)
                faulty_depth_image = np.clip(blurred_image + noise, 0, 65535)

            else:
                rospy.logwarn("Invalid fault type. Publishing original depth image.")
                faulty_depth_image = depth_image

            # Convert back to ROS Image message
            faulty_msg = self.bridge.cv2_to_imgmsg(faulty_depth_image, encoding="passthrough")
            faulty_msg.header = msg.header  # Preserve original timestamp

            # Publish the faulty depth image
            self.pub.publish(faulty_msg)

        except Exception as e:
            rospy.logerr("Error in depth fault injection: %s", str(e))

if __name__ == "__main__":
    # Parse command-line arguments for fault type selection
    parser = argparse.ArgumentParser(description="Inject faults into depth image")
    parser.add_argument("fault_type", choices=["blur", "noise", "both"], help="Type of fault to inject")
    args = parser.parse_args()

    DepthFaultInjector(args.fault_type)
    rospy.spin()

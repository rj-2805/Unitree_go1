#!/usr/bin/env python

import rospy
import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from random import uniform, randint
import time

class ImageFaultInjector:
    def __init__(self, fault_type):
        rospy.init_node("image_fault_injector", anonymous=True)
        self.bridge = CvBridge()
        self.fault_type = fault_type
        self.start_time = time.time()
        self.fault_duration = 240  # seconds

        # Set fault parameters
        if self.fault_type == "Gaussian_noise":
            self.std_dev = uniform(0.5, 1.5)
            rospy.logwarn("Gaussian noise STD_DEV: %f", self.std_dev)

        elif self.fault_type == "Image_blur":
            self.kernel_size = randint(30, 50)
            if self.kernel_size % 2 == 0:
                self.kernel_size += 1  # Ensure odd kernel size for OpenCV
            rospy.logwarn("Image blur kernel size: %d", self.kernel_size)

        elif self.fault_type == "Brightness_shift":
            self.beta = randint(50, 150)
            rospy.logwarn("Brightness shift beta: %d", self.beta)

        elif self.fault_type == "Motion_blur":
            self.kernel_size = randint(30, 50)
            if self.kernel_size % 2 == 0:
                self.kernel_size += 1  # Ensure odd kernel size
            self.angle = randint(0, 180)
            rospy.logwarn("Motion blur kernel size: %d, angle: %d", self.kernel_size, self.angle)

        else:
            rospy.logerr("Invalid fault type: %s", self.fault_type)
            sys.exit(1)

        # ROS Subscriber & Publisher
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/camera/color/faulty_image", Image, queue_size=10)

    def apply_gaussian_noise(self, image):
        noise = np.random.normal(0, self.std_dev, image.shape).astype(np.uint8)
        return cv2.add(image, noise)

    def apply_image_blur(self, image):
        return cv2.GaussianBlur(image, (self.kernel_size, self.kernel_size), 0)

    def apply_brightness_shift(self, image):
        return cv2.convertScaleAbs(image, alpha=1, beta=self.beta)

    def apply_motion_blur(self, image):
        kernel = np.zeros((self.kernel_size, self.kernel_size))
        center = (self.kernel_size - 1) / 2
        angle_rad = np.radians(self.angle)

        for i in range(self.kernel_size):
            x = i - center
            y = int(round(center + (x - center) * np.tan(angle_rad)))
            if 0 <= x < self.kernel_size and 0 <= y < self.kernel_size:
                kernel[int(x), int(y)] = 1

        kernel /= np.sum(kernel)
        return cv2.filter2D(image, -1, kernel)

    def image_callback(self, msg):
        current_time = time.time()
        inject_fault = current_time - self.start_time < self.fault_duration

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if inject_fault:
                if self.fault_type == "Gaussian_noise":
                    faulty_image = self.apply_gaussian_noise(cv_image)
                elif self.fault_type == "Image_blur":
                    faulty_image = self.apply_image_blur(cv_image)
                elif self.fault_type == "Brightness_shift":
                    faulty_image = self.apply_brightness_shift(cv_image)
                elif self.fault_type == "Motion_blur":
                    faulty_image = self.apply_motion_blur(cv_image)
                else:
                    faulty_image = cv_image
            else:
                faulty_image = cv_image  # No fault after 2 minutes

            faulty_msg = self.bridge.cv2_to_imgmsg(faulty_image, encoding="bgr8")
            faulty_msg.header.stamp = rospy.Time.now()
            self.image_pub.publish(faulty_msg)

        except Exception as e:
            rospy.logerr("Error in fault injection: %s", str(e))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        rospy.logerr("Usage: rosrun your_package image_fault_injector.py <fault_type>")
        sys.exit(1)

    fault_type = sys.argv[1]
    ImageFaultInjector(fault_type)
    rospy.spin()

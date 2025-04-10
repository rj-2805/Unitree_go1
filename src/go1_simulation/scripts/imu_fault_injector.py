#!/usr/bin/env python

import rospy
import random
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32


class IMUFaultInjector:
    def __init__(self):
        rospy.init_node('imu_fault_injector', anonymous=True)

        # Subscribe to original IMU data
        self.sub = rospy.Subscriber('/imu/data', Imu, self.inject_fault)

        # Publish to a new topic with faulty data
        self.pub = rospy.Publisher('/imu/faulty_data', Imu, queue_size=10)

        # Fault type (choose one or modify as needed)
        self.fault_type = rospy.get_param("~fault_type", "noise")  # Options: 'noise', 'delay', 'freeze', 'drift', 'corrupt'

    
        self.frozen_data = None  # for freeze fault

   
    def inject_fault(self, msg):
        faulty_msg = Imu()
        faulty_msg.header = msg.header

        # Default values (copied)
        faulty_msg.orientation = msg.orientation
        faulty_msg.angular_velocity = msg.angular_velocity
        faulty_msg.linear_acceleration = msg.linear_acceleration

        if self.fault_type == "noise":
            # Add noise to orientation and acceleration
            faulty_msg.orientation.x += random.uniform(-5.0, 5.0)
            faulty_msg.orientation.y += random.uniform(-5.0, 5.0)
            faulty_msg.orientation.z += random.uniform(-5.0, 5.0)
            faulty_msg.orientation.w += random.uniform(-5.0, 5.0)

            faulty_msg.linear_acceleration.x += random.uniform(-5.0, 5.0)
            faulty_msg.linear_acceleration.y += random.uniform(-5.0, 5.0)
            faulty_msg.linear_acceleration.z += random.uniform(-5.0, 5.0)

        elif self.fault_type == "delay":
            rospy.sleep(0.5)

        elif self.fault_type == "freeze":
            if self.frozen_data is None:
                self.frozen_data = msg
            faulty_msg = self.frozen_data  # Re-publish the same old value

        elif self.fault_type == "drift":
            # Slowly increasing drift in acceleration
            drift = rospy.get_time() * 0.01
            faulty_msg.linear_acceleration.x += drift
            faulty_msg.linear_acceleration.y += drift
            faulty_msg.linear_acceleration.z += drift

        elif self.fault_type == "corrupt":
            # Completely wrong random values
            faulty_msg.linear_acceleration.x = random.uniform(-100, 100)
            faulty_msg.angular_velocity.z = random.uniform(-50, 50)

        # Publish the faulty IMU data
        self.pub.publish(faulty_msg)


if __name__ == '__main__':
    try:
        IMUFaultInjector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

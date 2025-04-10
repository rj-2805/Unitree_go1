#!/usr/bin/env python

import rospy
import random
import time
import sys
from champ_msgs.msg import ContactsStamped
from std_msgs.msg import Int32


class FootContactFaultInjector:
    def __init__(self, fault_type):
        rospy.init_node("foot_contact_fault_injector", anonymous=True)
        
        self.fault_type = fault_type
        self.drop_rate = 0.3  # 30% message drop rate
        self.delay_time = 0.5  # 0.5-second delay
        self.noise_level = 0.1  # Noise level for contacts
        
        # Publisher for faulty contact data
        self.faulty_pub = rospy.Publisher("/faulty_foot_contacts", ContactsStamped, queue_size=10)

        # Subscribe to original contact data
        rospy.Subscriber("/foot_contacts", ContactsStamped, self.fault_injection_callback)

        rospy.loginfo(f"Foot Contact Fault Injector started with mode: {self.fault_type}")

        self.current_marker = None  # Holds current target marker
        self.marker_sub = rospy.Subscriber("/current_marker_goal", Int32, self.marker_callback)

    def marker_callback(self, msg):
        self.current_marker = msg.data

    def fault_injection_callback(self, msg):
        """ Modify the contact data based on the fault type """
        
        if self.fault_type == "drop" and self.current_marker == 2:
            if random.random() < self.drop_rate:
                rospy.logwarn("Dropping contact data (Simulated Fault)")
                return  # Drop the message

        elif self.fault_type == "delay" and self.current_marker == 2:
            rospy.logwarn("Delaying contact data (Simulated Fault)")
            time.sleep(self.delay_time)

        elif self.fault_type == "noise" and self.current_marker == 2:
            rospy.logwarn("Adding noise to contact data")
            for i in range(len(msg.contacts)):
                msg.contacts[i] = bool(random.choice([True, False]))  # Flip contact states randomly

        elif self.fault_type == "failure" and self.current_marker == 2:
            rospy.logwarn("Simulating sensor failure (fl contacts set to False)")
            msg.contacts[0] = False   # Set fl contacts to False

        # Publish modified message
        self.faulty_pub.publish(msg)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: rosrun <your_package> foot_contact_fault_injector.py <fault_type>")
        print("Fault Types: noise, drop, delay, failure")
        sys.exit(1)

    fault_type = sys.argv[1]
    if fault_type not in ["noise", "drop", "delay", "failure"]:
        print("Invalid fault type! Choose from: noise, drop, delay, failure")
        sys.exit(1)

    injector = FootContactFaultInjector(fault_type)
    rospy.spin()

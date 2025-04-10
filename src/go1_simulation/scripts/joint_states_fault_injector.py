#!/usr/bin/env python

import rospy
import random
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32


class JointStateFaultInjector:
    def __init__(self):
        rospy.init_node('joint_state_fault_injector', anonymous=True)

        # Subscribe to original /joint_states
        self.sub = rospy.Subscriber('/joint_states', JointState, self.inject_fault)

        # Publish to a new topic with faulty data
        self.pub = rospy.Publisher('/faulty_joint_states', JointState, queue_size=10)

        # Fault type (choose one or modify as needed)
        self.fault_type = rospy.get_param("~fault_type", "noise")  # Options: 'noise', 'delay', 'stuck', 'corrupt'

        self.current_marker = None  # Track the current goal marker
        self.marker_sub = rospy.Subscriber('/current_marker_goal', Int32, self.marker_callback)
        self.joint_index = 6 # fault injection in RL_calfjoint  

    def marker_callback(self, msg):
        self.current_marker = msg.data  #  Store current marker


    def inject_fault(self, msg):
        """Modify the incoming joint states message to inject faults"""
        faulty_msg = JointState()
        faulty_msg.header = msg.header
        faulty_msg.name = msg.name
        faulty_msg.position = list(msg.position)  # Copy original positions
        faulty_msg.velocity = list(msg.velocity)  # Copy original velocities
        faulty_msg.effort = list(msg.effort)      # Copy original efforts

        if self.fault_type == "noise" and self.current_marker == 1:
            for i in range(len(faulty_msg.position)):
                faulty_msg.position[self.joint_index] += random.uniform(-0.1, 0.1)

        elif self.fault_type == "delay" and self.current_marker == 1:
            rospy.sleep(0.5)  # Introduce artificial delay (500ms)

        elif self.fault_type == "stuck" and self.current_marker == 1:
            # Keep one joint position constant
            if len(faulty_msg.position) > 0:
                faulty_msg.position[0] = 1.57  # Force a joint to a fixed value

        elif self.fault_type == "corrupt" and self.current_marker == 1:
            # Set random incorrect values to some joints
            for i in range(len(faulty_msg.position)):
                if random.random() < 0.3:  # 30% chance of corruption
                    faulty_msg.position[i] = random.uniform(-3.14, 3.14)  # Random wrong value

        # Publish the faulty joint states
        self.pub.publish(faulty_msg)

if __name__ == '__main__':
    try:
        JointStateFaultInjector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

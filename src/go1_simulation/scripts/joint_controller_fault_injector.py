#!/usr/bin/env python3

import rospy
import random
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Int32


class FaultInjector:
    def __init__(self):
        rospy.init_node('fault_injector', anonymous=True)

        # Subscribe to the original joint command topic
        self.sub = rospy.Subscriber('/joint_group_position_controller/command', JointTrajectory, self.inject_fault)

        # Publish to the faulty joint command topic
        self.pub = rospy.Publisher('/faulty_joint_group_position_controller/command', JointTrajectory, queue_size=10)

        # Get fault type from ROS parameter
        self.fault_type = rospy.get_param("~fault_type", "noise")  # Options: 'noise', 'delay', 'stuck', 'corrupt'

        rospy.loginfo(f"Fault Injector Initialized with Fault Type: {self.fault_type}")

        self.current_marker = None  # Track current target marker
        self.marker_sub = rospy.Subscriber('/current_marker_goal', Int32, self.marker_callback)

        self.joint_index = 6  # fault injection in RL_calfjoint

    def marker_callback(self, msg):
        self.current_marker = msg.data

    def inject_fault(self, msg):
        """ Modify incoming joint trajectory messages to inject faults """

        faulty_msg = JointTrajectory()
        faulty_msg.header = msg.header
        faulty_msg.joint_names = msg.joint_names

        for point in msg.points:
            faulty_point = JointTrajectoryPoint()
            faulty_point.positions = list(point.positions)  # Copy original positions
            faulty_point.velocities = list(point.velocities) if point.velocities else []
            faulty_point.accelerations = list(point.accelerations) if point.accelerations else []
            faulty_point.effort = list(point.effort) if point.effort else []
            faulty_point.time_from_start = point.time_from_start

            if self.fault_type == "noise" and self.current_marker == 3:
                rospy.logwarn_once("Injecting Noise into Joint Positions")
                # Add small random noise to each joint position
                faulty_point.positions[self.joint_index] =+ random.uniform(-1.0, 1.0)

            elif self.fault_type == "delay" and self.current_marker == 3:
                rospy.logwarn_once("Injecting Delay in Command Processing")
                rospy.sleep(0.5)  # Introduce artificial delay (500ms)

            elif self.fault_type == "stuck" and self.current_marker == 3:
                rospy.logwarn_once("Injecting Stuck Joint Fault")
                if faulty_point.positions:
                    faulty_point.positions[0] = 1.57  # Fix one joint to a constant value

            elif self.fault_type == "corrupt" and self.current_marker == 3:
                rospy.logwarn_once("Injecting Random Corruption in Joint Positions")
                for i in range(len(faulty_point.positions)):
                    if random.random() < 0.3:  # 30% chance of corruption
                        faulty_point.positions[i] = random.uniform(-3.14, 3.14)  # Random wrong value

            faulty_msg.points.append(faulty_point)

        # Publish the faulty command
        self.pub.publish(faulty_msg)

if __name__ == '__main__':
    try:
        FaultInjector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

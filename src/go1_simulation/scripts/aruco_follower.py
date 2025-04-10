#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int32MultiArray
import math
import time

# Parameters for controlling robot movement
MAX_LINEAR_SPEED = 0.5  # Maximum linear speed (m/s)
MAX_ANGULAR_SPEED = 1.0  # Maximum angular speed (rad/s)
DISTANCE_TOLERANCE = 0.1  # Tolerance for distance to stop moving (m)
ANGLE_TOLERANCE = 0.05  # Tolerance for angular alignment (rad)

# Desired marker visiting order
MARKER_ORDER = [1, 2, 3, 4]


class ArucoSequentialFollower:
    def __init__(self):
        rospy.init_node("aruco_sequential_follower", anonymous=True)

        # Initialize ROS publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/aruco_marker_pose", Pose, self.pose_callback)
        rospy.Subscriber("/aruco_marker_ids", Int32MultiArray, self.marker_id_callback)

        self.marker_poses = {}  # Store poses of detected markers {marker_id: Pose}
        self.current_marker_id = None
        self.marker_ids = []  # List of currently detected marker IDs
        rospy.loginfo("Aruco Sequential Follower Node Initialized")

    def pose_callback(self, pose_msg):
        """Callback for the ArUco marker pose topic."""
        if self.current_marker_id in self.marker_ids:
            # Update pose of the current target marker
            self.marker_poses[self.current_marker_id] = pose_msg

    def marker_id_callback(self, marker_ids_msg):
        """Callback for the marker IDs topic."""
        self.marker_ids = marker_ids_msg.data

    def move_to_marker(self, target_id):
        """Moves the robot to the marker with the given ID."""
        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            if target_id in self.marker_ids and target_id in self.marker_poses:
                # Get marker position
                marker_pose = self.marker_poses[target_id]
                marker_x = marker_pose.position.x
                marker_y = marker_pose.position.y
                marker_z = marker_pose.position.z

                # Calculate distance and angle to the marker
                distance = (marker_x**2 + marker_y**2)**0.5
                angle_to_marker = math.atan2(marker_y, marker_x)

                rospy.loginfo(f"Target Marker {target_id}: Distance = {distance:.2f} m, Angle = {angle_to_marker:.2f} rad")

                # Create a Twist message for velocity control
                cmd = Twist()

                if distance > DISTANCE_TOLERANCE:
                    # Linear velocity
                    cmd.linear.x = min(MAX_LINEAR_SPEED, distance)

                    # Angular velocity for alignment
                    if abs(angle_to_marker) > ANGLE_TOLERANCE:
                        cmd.angular.z = max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, angle_to_marker))
                    else:
                        cmd.angular.z = 0.0
                else:
                    # Stop if within distance tolerance
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.cmd_vel_pub.publish(cmd)
                    rospy.loginfo(f"Reached Marker {target_id}")
                    return  # Exit the function when the target is reached

                # Publish the velocity command
                self.cmd_vel_pub.publish(cmd)

            else:
                rospy.loginfo(f"Marker {target_id} not detected yet. Waiting...")
                time.sleep(0.5)

            rate.sleep()

    def start_sequence(self):
        """Starts the sequence to visit markers in the desired order."""
        for marker_id in MARKER_ORDER:
            rospy.loginfo(f"Moving to Marker {marker_id}")
            self.current_marker_id = marker_id
            self.move_to_marker(marker_id)
            rospy.loginfo(f"Completed Marker {marker_id}")


if __name__ == "__main__":
    try:
        follower = ArucoSequentialFollower()
        follower.start_sequence()
    except rospy.ROSInterruptException:
        rospy.loginfo("Aruco Sequential Follower Node Terminated")

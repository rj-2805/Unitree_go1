#!/usr/bin/env python3

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from go1_simulation.msg import ArucoMarker
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans
from math import atan2


class ArucoToMoveBase:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("aruco_to_move_base", anonymous=True)

        # Publisher for the /move_base/goal topic
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)

        # Subscriber to the /aruco_marker topic
        rospy.Subscriber("/aruco_marker", ArucoMarker, self.aruco_callback)

        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Aruco to MoveBase Node Started")

    def get_navigation_quaternion(self, pose):
        """
        Extracts yaw from a quaternion and returns a valid navigation quaternion.
        """
        # Calculate yaw (rotation around Z-axis)
        yaw = atan2(
            2.0 * (pose.orientation.w * pose.orientation.z),
            1.0 - 2.0 * (pose.orientation.z * pose.orientation.z)
        )

        # Convert yaw back to a navigation-compatible quaternion
        q = tf_trans.quaternion_from_euler(0, 0, yaw)
        return q

    def transform_pose_to_map(self, pose_stamped):
        """
        Transforms the pose from the trunk frame to the map frame.
        """
        try:
            # Look up the transform from trunk to map
            transform = self.tf_buffer.lookup_transform("map", "trunk", rospy.Time(0), rospy.Duration(1.0))

            # Perform the transformation
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            return transformed_pose

        except Exception as e:
            rospy.logerr(f"Error transforming pose to map frame: {e}")
            return None

    def aruco_callback(self, marker_msg):
        """
        Callback function to handle incoming ArucoMarker messages.
        """
        rospy.loginfo(f"Received ArUco Marker ID: {marker_msg.id}")

        # Create a PoseStamped from the ArucoMarker pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "trunk"  # The pose is in the trunk frame
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = marker_msg.pose

        # Transform the pose to the map frame
        transformed_pose = self.transform_pose_to_map(pose_stamped)

        if transformed_pose is None:
            rospy.logwarn("Skipping goal publication due to transform failure.")
            return

        # Calculate the direction vector to stop 0.5m in front of the marker
        marker_position = transformed_pose.pose.position
        direction_x = marker_position.x
        direction_y = marker_position.y
        magnitude = (direction_x**2 + direction_y**2)**0.5

        # Normalize the direction vector and scale it by 0.5m
        stop_distance = 0.5
        adjusted_x = marker_position.x - (direction_x / magnitude) * stop_distance
        adjusted_y = marker_position.y - (direction_y / magnitude) * stop_distance

        # Update the position to stop 0.5m in front of the marker
        adjusted_position = transformed_pose.pose.position
        adjusted_position.x = adjusted_x
        adjusted_position.y = adjusted_y

        # Extract position and adjust the quaternion for navigation
        valid_quaternion = self.get_navigation_quaternion(transformed_pose.pose)

        # Create a MoveBaseActionGoal message
        goal_msg = MoveBaseActionGoal()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"  # The pose is now in the map frame

        # Assign the adjusted pose to the goal message
        goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
        goal_msg.goal.target_pose.header.frame_id = "map"
        goal_msg.goal.target_pose.pose.position = adjusted_position
        goal_msg.goal.target_pose.pose.orientation.x = valid_quaternion[0]
        goal_msg.goal.target_pose.pose.orientation.y = valid_quaternion[1]
        goal_msg.goal.target_pose.pose.orientation.z = valid_quaternion[2]
        goal_msg.goal.target_pose.pose.orientation.w = valid_quaternion[3]

        # Publish the goal message to /move_base/goal
        self.goal_pub.publish(goal_msg)
        rospy.loginfo(f"Published goal for Marker ID {marker_msg.id} to /move_base/goal (adjusted to stop 0.5m in front)")


if __name__ == "__main__":
    try:
        node = ArucoToMoveBase()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

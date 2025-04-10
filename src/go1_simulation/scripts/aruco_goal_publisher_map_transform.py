#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from move_base_msgs.msg import MoveBaseActionGoal
from go1_simulation.msg import ArucoMarker
from geometry_msgs.msg import PoseStamped

class ArucoToMoveBase:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("aruco_to_move_base", anonymous=True)

        # Publisher for the /move_base/goal topic
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseActionGoal, queue_size=10)

        # Subscriber to the /aruco_marker topic
        rospy.Subscriber("/aruco_marker", ArucoMarker, self.aruco_callback)

        # TF Buffer and Listener for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("Aruco to MoveBase Node Started")

    def aruco_callback(self, marker_msg):
        """
        Callback function to handle incoming ArucoMarker messages.
        """
        rospy.loginfo(f"Received ArUco Marker ID: {marker_msg.id}")

        # Create a PoseStamped message for the marker pose
        trunk_pose = PoseStamped()
        trunk_pose.header.frame_id = "trunk"  # Current frame of the ArucoMarker pose
        trunk_pose.header.stamp = rospy.Time.now()
        trunk_pose.pose = marker_msg.pose

        try:
            # Transform the pose from 'trunk' to 'map'
            transform = self.tf_buffer.lookup_transform("map", "trunk", rospy.Time(0), rospy.Duration(2.0))
            map_pose = tf2_geometry_msgs.do_transform_pose(trunk_pose, transform)

            # Create a MoveBaseActionGoal message
            goal_msg = MoveBaseActionGoal()

            # Set the header and frame_id
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = "map"

            # Assign the transformed pose to the goal message
            goal_msg.goal.target_pose.header.stamp = rospy.Time.now()
            goal_msg.goal.target_pose.header.frame_id = "map"
            goal_msg.goal.target_pose.pose = map_pose.pose

            # Publish the goal message to /move_base/goal
            self.goal_pub.publish(goal_msg)
            rospy.loginfo(f"Published goal for Marker ID {marker_msg.id} to /move_base/goal")

        except tf2_ros.LookupException as e:
            rospy.logerr(f"Transform lookup error: {e}")
        except tf2_ros.ConnectivityException as e:
            rospy.logerr(f"Transform connectivity error: {e}")
        except tf2_ros.ExtrapolationException as e:
            rospy.logerr(f"Transform extrapolation error: {e}")

if __name__ == "__main__":
    try:
        node = ArucoToMoveBase()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

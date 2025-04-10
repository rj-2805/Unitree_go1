#!/usr/bin/env python3

import os
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Int32


stop_distance = 1.0

class SequentialGoalPublisher:
    def __init__(self, yaml_file_path):
        self.yaml_file_path = yaml_file_path
        self.goal_poses = []
        self.goal_index = 0
        self.goal_reached = False

        # ROS Node
        rospy.init_node('sequential_goal_publisher', anonymous=True)

        # Publisher
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Subscriber for feedback from the navigation stack
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        self.marker_pub = rospy.Publisher('/current_marker_goal', Int32, queue_size=10)


    def find_yaml_file(self, file_name, search_path):
        """
        Search for a YAML file recursively in the specified directory.
        :param file_name: Name of the YAML file to find.
        :param search_path: Path to start the search from.
        :return: Full path to the file if found, else None.
        """
        for root, dirs, files in os.walk(search_path):
            if file_name in files:
                return os.path.join(root, file_name)
        return None

    def load_poses_from_yaml(self):
        """
        Load poses from a YAML file.
        """
        with open(self.yaml_file_path, 'r') as file:
            try:
                data = yaml.safe_load(file)
                self.goal_poses = data['poses']
            except Exception as e:
                rospy.logerr(f"Error loading YAML file: {e}")
                self.goal_poses = []

    def status_callback(self, status_msg):
        """
        Callback for checking the goal status.
        """
        if not status_msg.status_list:
            return

        # Only check the latest goal's status
        last_status = status_msg.status_list[-1].status

        # Ignore old statuses at startup
        if self.goal_index == 0 and last_status == 3:
            rospy.logwarn("Ignoring previous success status at startup.")
            return  

        # GoalStatus: SUCCEEDED = 3
        if last_status == 3:
            rospy.loginfo("Goal reached successfully!")
            self.goal_reached = True

    def publish_next_goal(self):
        """
        Publish the next goal pose if available.
        """
        if self.goal_index >= len(self.goal_poses):
            rospy.loginfo("All goals have been published and reached.")
            return False

        pose_data = self.goal_poses[self.goal_index]
        pose = PoseStamped()
        pose.header.frame_id = "map"  # Replace with your frame of reference
        pose.header.stamp = rospy.Time.now()

        # Set position
        pose.pose.position.x = pose_data['position']['x']
        pose.pose.position.y = pose_data['position']['y']
        pose.pose.position.z = pose_data['position']['z']

        # To stop the robot infront of the marker
        if pose.pose.position.x > 0:
            if pose.pose.position.y < 0:
                pose.pose.position.y = pose.pose.position.y + stop_distance
            elif pose.pose.position.y > 0:
                pose.pose.position.y = pose.pose.position.y - stop_distance
        elif pose.pose.position.x < 0:
            pose.pose.position.x = pose.pose.position.x + stop_distance

        # Set orientation
        pose.pose.orientation.x = pose_data['orientation']['x']
        pose.pose.orientation.y = pose_data['orientation']['y']
        pose.pose.orientation.z = pose_data['orientation']['z']
        pose.pose.orientation.w = pose_data['orientation']['w']

         # Publish the current marker ID
        marker_id = pose_data.get('id', -1)
        self.marker_pub.publish(marker_id)

        rospy.loginfo(f"Publishing goal pose ID: {pose_data.get('id', 'unknown')}")
        self.pub.publish(pose)
        self.goal_index += 1
        return True

    def run(self):
        """
        Main loop to publish goals and wait until the prior goal is reached.
        """
        self.load_poses_from_yaml()

        if not self.goal_poses:
            rospy.logerr("No poses found in the YAML file. Exiting.")
            return

        rospy.loginfo("Starting sequential goal publishing...")

        # Small delay before sending the first goal to clear previous statuses
        rospy.sleep(2)

        while not rospy.is_shutdown():
            if self.goal_reached or self.goal_index == 0:  # If first goal or previous goal reached
                self.goal_reached = False
                if not self.publish_next_goal():
                    break  # Stop if no more goals are left

            rospy.sleep(2)  # Allow status updates


if __name__ == '__main__':
    try:
        # File name of the YAML file
        yaml_file_name = "markers.yaml"

        # Directory to search (current working directory)
        search_directory = os.getcwd()

        # Find the YAML file
        yaml_file_path = SequentialGoalPublisher.find_yaml_file(None, yaml_file_name, search_directory)

        if yaml_file_path:
            rospy.loginfo(f"Found YAML file at: {yaml_file_path}")

            # Initialize and run the sequential goal publisher
            node = SequentialGoalPublisher(yaml_file_path)
            node.run()
        else:
            rospy.logerr(f"YAML file '{yaml_file_name}' not found in directory: {search_directory}")

    except rospy.ROSInterruptException:
        rospy.loginfo("Sequential goal publisher node interrupted.")
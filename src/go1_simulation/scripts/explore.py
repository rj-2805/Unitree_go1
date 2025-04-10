#!/usr/bin/env python3

from typing import Optional, List
import tf2_geometry_msgs
import yaml

import actionlib
import numpy as np
import rospy
from geometry_msgs.msg import (
    Pose,
    PoseArray,
    PoseStamped,
    Twist,
    PoseWithCovarianceStamped,
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid
from skimage.morphology import flood_fill
from actionlib_msgs.msg import GoalStatus
from go1_simulation.msg import ArucoMarker
from pathlib import Path
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import atan2


def average_pose(poses: List[PoseStamped]):
    positions = np.array(
        [(p.pose.position.x, p.pose.position.y, p.pose.position.z) for p in poses]
    )
    rotations = np.array(
        [
            euler_from_quaternion(
                [
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                ]
            )
            for p in poses
        ]
    )
    a_position = np.mean(positions, axis=0)
    a_rotation = quaternion_from_euler(*np.mean(rotations, axis=0))

    return a_position.tolist(), a_rotation.tolist()


class ArucoSaver:
    def __init__(self, expected_nb_ids=4, save_path: Path = Path("/home/franka/robodog_janavi/rp_final/src/go1_simulation/yaml/markers.yaml")):
        self.expected_nb_ids = expected_nb_ids
        self.found = {}
        self.save_path = Path(save_path)
        self.sub = rospy.Subscriber("aruco_marker", ArucoMarker, self.add_marker)
        self.min_markers_to_save = rospy.get_param("min_nb_markers", 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

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
        q = quaternion_from_euler(0, 0, yaw)
        return q

    # def add_marker(self, marker: ArucoMarker):
    #     print(f"got marker {marker.id}")
    #     if marker.id not in self.found:
    #         self.found[marker.id] = []

    #     relative_marker_pose = PoseStamped()
    #     relative_marker_pose.header = marker.header
    #     relative_marker_pose.pose = marker.pose
    #     transform = self.tf_buffer.lookup_transform(
    #         "map", marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
    #     )
    #     absolute_marker_pose = tf2_geometry_msgs.do_transform_pose(
    #         relative_marker_pose, transform
    #     )
    #     self.found[marker.id].append(absolute_marker_pose)

    #     self.save_markers()

    def add_marker(self, marker: ArucoMarker):
        print(f"got marker {marker.id}")
        if marker.id not in self.found:
            self.found[marker.id] = []

        relative_marker_pose = PoseStamped()
        relative_marker_pose.header = marker.header
        relative_marker_pose.pose = marker.pose
        try:
            # Transform the marker pose to the map frame
            transform = self.tf_buffer.lookup_transform(
                "map", marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0)
            )
            absolute_marker_pose = tf2_geometry_msgs.do_transform_pose(
                relative_marker_pose, transform
            )

            # Extract and normalize yaw quaternion for navigation
            navigation_quaternion = self.get_navigation_quaternion(absolute_marker_pose.pose)
            absolute_marker_pose.pose.orientation.x = navigation_quaternion[0]
            absolute_marker_pose.pose.orientation.y = navigation_quaternion[1]
            absolute_marker_pose.pose.orientation.z = navigation_quaternion[2]
            absolute_marker_pose.pose.orientation.w = navigation_quaternion[3]

            # Store the transformed pose
            self.found[marker.id].append(absolute_marker_pose)

            self.save_markers()
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed for marker {marker.id}: {e}")


    def found_all(self):
        enough_markers_found = {
            k: len(v)
            for k, v in self.found.items()
            if len(v) >= self.min_markers_to_save
        }
        return len(enough_markers_found) >= self.expected_nb_ids

    def save_markers(self):
        yaml_out = {"poses": []}
        for id, poses in self.found.items():
            if len(poses) < self.min_markers_to_save:
                continue

            print(f"id has a enough of poses: {len(poses)}")
            p, q = average_pose(poses)
            d = {
                "id": id,
                "position": {
                    "x": p[0],
                    "y": p[1],
                    "z": p[2],
                },
                "orientation": {
                    "x": q[0],
                    "y": q[1],
                    "z": q[2],
                    "w": q[3],
                },
            }
            yaml_out["poses"].append(d)

        with self.save_path.open("w") as f:
            print(yaml_out)
            yaml.dump(yaml_out, f, Dumper=yaml.SafeDumper)
            print("dump over")


class VoxelGrid2D:
    def __init__(self, points, voxel_size):
        """
        Initializes the voxel grid.

        :param points: Nx2 numpy array of (x, y) points.
        :param voxel_size: The size of each voxel.
        """
        self.voxel_size = voxel_size
        self.voxel_map = {}  # Dictionary to store voxel data

        # Compute grid bounds
        min_x, min_y = np.min(points, axis=0) - voxel_size
        max_x, max_y = np.max(points, axis=0) + voxel_size

        self.origin = np.array([min_x, min_y])  # Store grid origin
        self.dimensions = np.ceil(
            np.array([max_x - min_x, max_y - min_y]) / voxel_size
        ).astype(int)

        # Build voxel grid
        self.build_grid(points)

    def build_grid(self, points):
        """
        Populates the voxel grid with points.

        :param points: Nx2 numpy array of (x, y) points.
        """
        for point in points:
            voxel_idx = self._get_voxel_index(point)
            if voxel_idx not in self.voxel_map:
                self.voxel_map[voxel_idx] = {"points": [], "traversed": False}
            self.voxel_map[voxel_idx]["points"].append(point)

    def _get_voxel_index(self, point):
        """
        Computes the voxel index for a given point.

        :param point: (x, y) numpy array.
        :return: Tuple (i, j) voxel index.
        """
        return tuple(np.floor((point - self.origin) / self.voxel_size).astype(int))

    def get_voxel_center(self, voxel_idx):
        """
        Computes the center of a voxel given its index.

        :param voxel_idx: Tuple (i, j).
        :return: (x, y) center coordinates of the voxel.
        """
        return self.origin + (np.array(voxel_idx) + 0.5) * self.voxel_size

    def mark_traversed(self, point):
        """
        Marks the voxel containing a given point as traversed.

        :param point: (x, y) numpy array.
        """
        voxel_idx = self._get_voxel_index(point)
        if voxel_idx in self.voxel_map:
            self.voxel_map[voxel_idx]["traversed"] = True
            return True
        return False

    def get_untraversed_voxels(self):
        """
        Returns a list of untraversed voxel centers.

        :return: List of (x, y) voxel center coordinates.
        """
        return [
            self.get_voxel_center(idx)
            for idx, voxel in self.voxel_map.items()
            if not voxel["traversed"]
        ]

    def get_voxel_count(self, voxel_idx):
        """
        Returns the number of points inside a given voxel.

        :param voxel_idx: Tuple (i, j).
        :return: Number of points in the voxel.
        """
        return len(self.voxel_map.get(voxel_idx, {}).get("points", []))

    def sort_untraversed_voxels_zigzag(self):
        untraversed_voxel_indices = self.get_untraversed_voxels()

        if not untraversed_voxel_indices:
            return []

        min_idx = np.min(untraversed_voxel_indices, axis=0)
        max_idx = np.max(untraversed_voxel_indices, axis=0)
        width, height = max_idx - min_idx

        sort_by_x = width >= height

        if sort_by_x:
            untraversed_voxel_indices.sort(
                key=lambda v: (v[0], v[1] if v[0] % 2 == 0 else -v[1])
            )
        else:
            untraversed_voxel_indices.sort(
                key=lambda v: (v[1], v[0] if v[1] % 2 == 0 else -v[0])
            )

        return untraversed_voxel_indices


class CoverageExplorer:
    def __init__(self):
        # Move base client
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base server ...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to move_base server.")

        # Publisher for rotation commands
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # voxel grid
        self.vg: Optional[VoxelGrid2D] = None
        # Subscribe to costmap
        self.coarse_dist = rospy.get_param("coarse_dist", 2)
        self.costmap: Optional[OccupancyGrid] = None

        self.costmap_sub = rospy.Subscriber(
            "/move_base/global_costmap/costmap", OccupancyGrid, self.costmap_callback
        )

        self.free_space_pub = rospy.Publisher(
            "free_space_goals", PoseArray, queue_size=1, latch=True
        )
        self.debug_pub = rospy.Publisher(
            "current_coverage_goal", PoseStamped, queue_size=1, latch=True
        )

        self.pose_sub = rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.update_robot_pose
        )

        self.aruco_saver = ArucoSaver()

    def costmap_callback(self, msg: OccupancyGrid):
        self.costmap = msg
        rospy.loginfo("got costmap, unregistering")
        self.costmap_sub.unregister()

        # Parse costmap
        width = self.costmap.info.width
        height = self.costmap.info.height
        resolution = self.costmap.info.resolution
        origin_x = self.costmap.info.origin.position.x
        origin_y = self.costmap.info.origin.position.y

        costmap_array = np.array(self.costmap.data).reshape((height, width))
        # flood fill interior
        costmap_array_inside = flood_fill(costmap_array, (300, 200), -10)

        free_indices = np.argwhere(costmap_array_inside == -10)

        free_coords = [
            [origin_x + i * resolution, origin_y + j * resolution]
            for i, j in free_indices
        ]

        # apply a voxel filter
        self.vg = VoxelGrid2D(free_coords, self.coarse_dist)

    def update_robot_pose(self, msg: PoseWithCovarianceStamped):
        if self.vg is None:
            return

        point = np.array([msg.pose.pose.position.y, msg.pose.pose.position.x])
        if self.vg.mark_traversed(point):
            rospy.loginfo(f"marking {point} as traversed")
            uv = self.vg.get_untraversed_voxels()
            self.pub_free_goals(uv)

    def send_goal(self, x, y):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        # goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = y
        goal.target_pose.pose.position.y = x
        goal.target_pose.pose.orientation.w = 1.0
        self.pub_current_goal(goal.target_pose.pose)

        rospy.loginfo(f"Sent goal to ({x}, {y})")
        res = self.client.send_goal_and_wait(goal, rospy.Duration(45))
        if res == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached !")
            assert self.vg is not None
            self.vg.mark_traversed([x, y])
            return True
        else:
            rospy.logwarn("failed to reach goal")
            return False

    def pub_current_goal(self, pose):
        pa = PoseStamped()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
        pa.pose = pose
        self.debug_pub.publish(pa)

    def pub_free_goals(self, list_xy):
        pa = PoseArray()
        pa.header.frame_id = "map"
        pa.header.stamp = rospy.Time.now()
        for x, y in list_xy:
            p = Pose()
            p.orientation.w = 1.0
            p.position.x = y
            p.position.y = x
            pa.poses.append(p)

        self.free_space_pub.publish(pa)

    def rotate_in_place(self):
        twist = Twist()
        twist.angular.z = 0.5

        duration = np.pi / twist.angular.z
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < duration:
            self.cmd_vel_pub.publish(twist)
            rospy.sleep(0.1)

        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def explore_area(self):
        while self.vg is None:
            rospy.logwarn("Waiting for costmap ...")
            rospy.sleep(1)

        uv = self.vg.get_untraversed_voxels()
        while len(uv) != 0:
            rospy.loginfo(f"Got {len(uv)} voxels left to explore")
            self.pub_free_goals(uv)
            # select a goal
            x, y = self.vg.sort_untraversed_voxels_zigzag()[0]
            # send the goal and wait with a timeout
            rospy.loginfo(f"Moving to ({x}, {y})")
            self.send_goal(x, y)
            if self.aruco_saver.found_all():
                rospy.loginfo("Found all markers")
                break
            rospy.sleep(1)
            rospy.loginfo("Rotating 180 degrees...")
            self.rotate_in_place()
            if self.aruco_saver.found_all():
                rospy.loginfo("Found all markers")
                break

            uv = self.vg.get_untraversed_voxels()


if __name__ == "__main__":
    rospy.init_node("coverage_explorer")
    explorer = CoverageExplorer()
    while not explorer.aruco_saver.found_all():
        explorer.explore_area()

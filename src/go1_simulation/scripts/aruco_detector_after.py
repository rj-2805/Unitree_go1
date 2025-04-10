import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from std_msgs.msg import Int32MultiArray
from go1_simulation.msg import ArucoMarker
from cv_bridge import CvBridge
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans  # Correct import for transformations

# Global Variables
bridge = CvBridge()
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()

# Camera Intrinsic Parameters (Replace with your camera calibration values)
camera_matrix = np.array([[462.1379699707031, 0.0, 320.0],
                          [0.0, 462.1379699707031, 240.0],
                          [0.0, 0.0, 1.0]], dtype=float)
fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]

dist_coeffs = np.array([0, 0, 0, 0], dtype=float)

# ROS Publishers
pose_pub = None
processed_image_pub = None
marker_id_pub = None
depth_image = None  # Latest depth image
aruco_marker_pub = None



def publish_static_transform():
    """
    Publishes the static transform between the camera frame and the base frame.
    """
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transform = TransformStamped()

    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "trunk"  # Base frame
    static_transform.child_frame_id = "camera_face"  # Camera frame

    # Set translation (meters)
    static_transform.transform.translation.x = 0.2785
    static_transform.transform.translation.y = 0.0125
    static_transform.transform.translation.z = 0.0167

    # Set rotation (roll-pitch-yaw to quaternion)
    q = tf_trans.quaternion_from_euler(0, 0, 0)  # Roll, pitch, yaw
    static_transform.transform.rotation.x = q[0]
    static_transform.transform.rotation.y = q[1]
    static_transform.transform.rotation.z = q[2]
    static_transform.transform.rotation.w = q[3]

    static_broadcaster.sendTransform(static_transform)
    rospy.loginfo("Static transform published: trunk -> camera_face")


def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logerr(f"Error converting depth image: {e}")

def adjust_marker_axes(rvec):
    """
    Adjust the marker's axes to match the trunk frame (Z up, Y forward).
    Applies a 90° rotation around the X-axis to align axes.
    :param rvec: Original rotation vector from cv2.solvePnP.
    :return: Adjusted rotation vector.
    """
    # 90° rotation around the X-axis
    R_corr = np.array([[1, 0, 0],
                       [0, 0, -1],
                       [0, 1, 0]])

    # Convert rvec to a rotation matrix
    R_original, _ = cv2.Rodrigues(rvec)

    # Apply the correction
    R_adjusted = R_corr @ R_original

    # Convert back to a rotation vector
    rvec_adjusted, _ = cv2.Rodrigues(R_adjusted)

    return rvec_adjusted

def process_image(msg):
    global bridge, aruco_dict, parameters, pose_pub, processed_image_pub, marker_id_pub, depth_image

    try:
        # Convert ROS Image message to OpenCV format
        frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception as e:
        rospy.logerr(f"Error converting RGB image: {e}")
        return

    # Convert to grayscale for ArUco detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None and depth_image is not None:
        marker_ids = Int32MultiArray()
        marker_ids.data = []

        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # Define the size of the marker axis (in meters)
        axis_length = 0.03  # Adjust this based on your marker size

        for i, marker_id in enumerate(ids.flatten()):
            # Compute the center of the marker in pixel coordinates
            marker_corners = corners[i][0]
            pixel_x = int(np.sum(marker_corners[:, 0]) / len(marker_corners[:, 0]))  # Mean of x-coordinates
            pixel_y = int(np.sum(marker_corners[:, 1]) / len(marker_corners[:, 1]))  # Mean of y-coordinates

            # Get the depth value at the marker's center
            try:
                Z = depth_image[pixel_y, pixel_x] / 1000.0  # Depth in meters (convert from mm)
            except IndexError:
                rospy.logerr("Pixel coordinates out of depth image bounds")
                continue

            if Z == 0:
                rospy.logwarn("Invalid depth value (0), skipping marker")
                continue

            # Calculate real-world X and Y in the camera frame
            X = (pixel_x - cx) * Z / fx
            Y = (pixel_y - cy) * Z / fy

            # Solve PnP to get the marker pose
            ret, rvec, tvec = cv2.solvePnP(
                np.array([[-0.025, -0.025, 0],
                          [ 0.025, -0.025, 0],
                          [ 0.025,  0.025, 0],
                          [-0.025,  0.025, 0]], dtype=float),  # Marker corners centered at origin
                marker_corners,
                camera_matrix,
                dist_coeffs
            )

            if ret:
                # Adjust the marker's axes to match the trunk frame
                rvec_adjusted = adjust_marker_axes(rvec)
                # Draw the marker axis on the frame
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec_adjusted, tvec, axis_length)

                # Log the calculated position in the camera frame
                rospy.loginfo(f"Marker ID {marker_id}: Camera Frame Pose: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}")

                # Create PoseStamped in the camera frame
                pose_stamped = PoseStamped()
                pose_stamped.header.frame_id = "camera_face"  # Camera frame
                pose_stamped.header.stamp = rospy.Time.now()
                pose_stamped.pose.position.x = X
                pose_stamped.pose.position.y = Y
                pose_stamped.pose.position.z = Z
                pose_stamped.pose.orientation.w = 1.0  # Identity quaternion

                # Transform pose to base frame
                try:
                    transform = tf_buffer.lookup_transform("trunk", "camera_face", rospy.Time(0), rospy.Duration(1.0))
                    base_pose_stamped = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

                    # Swap Y and Z coordinates to align with navigation stack conventions
                    corrected_pose = PoseStamped()
                    corrected_pose.header = base_pose_stamped.header
                    corrected_pose.pose.position.x = base_pose_stamped.pose.position.x
                    corrected_pose.pose.position.y = base_pose_stamped.pose.position.z  # Z -> Y
                    corrected_pose.pose.position.z = -base_pose_stamped.pose.position.y  # Y -> -Z
                    corrected_pose.pose.orientation = base_pose_stamped.pose.orientation

                    # Create a custom message for the detected marker in the trunk frame
                    marker_msg = ArucoMarker()
                    marker_msg.header = corrected_pose.header  # Use corrected pose header
                    marker_msg.id = marker_id
                    marker_msg.pose = corrected_pose.pose  # Corrected pose

                    # Publish the transformed marker message
                    aruco_marker_pub.publish(marker_msg)

                    # Log the position in the base frame
                    rospy.loginfo(f"Marker ID {marker_id}: Base Frame Pose: "
                                  f"X={corrected_pose.pose.position.x:.2f}, "
                                  f"Y={corrected_pose.pose.position.y:.2f}, "
                                  f"Z={corrected_pose.pose.position.z:.2f}")

                    # Publish the corrected pose
                    pose_pub.publish(corrected_pose.pose)

                except Exception as e:
                    rospy.logerr(f"Error transforming pose to base frame: {e}")
                    continue

        # Publish marker IDs
        marker_id_pub.publish(marker_ids)

    else:
        rospy.loginfo("No markers detected or depth image not available.")

    # Publish the processed image
    try:
        processed_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        processed_image_pub.publish(processed_image)
    except Exception as e:
        rospy.logerr(f"Error publishing processed image: {e}")


def main():
    global pose_pub, processed_image_pub, marker_id_pub, aruco_marker_pub


    rospy.init_node("aruco_pose_transform_node", anonymous=True)

    # Publish the static transform
    publish_static_transform()

    # Subscribe to RGB and Depth topics
    rospy.Subscriber("/camera/color/image_raw", Image, process_image)
    rospy.Subscriber("/camera/depth/image_raw", Image, depth_callback)

    # Publishers for marker pose, processed image, and marker IDs
    pose_pub = rospy.Publisher("/aruco_marker_pose", Pose, queue_size=10)
    processed_image_pub = rospy.Publisher("/aruco_processed_image", Image, queue_size=10)
    marker_id_pub = rospy.Publisher("/aruco_marker_ids", Int32MultiArray, queue_size=10)
    aruco_marker_pub = rospy.Publisher("/aruco_marker", ArucoMarker, queue_size=10)  # Custom publisher

    rospy.loginfo("Aruco Pose Transform Node Started")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

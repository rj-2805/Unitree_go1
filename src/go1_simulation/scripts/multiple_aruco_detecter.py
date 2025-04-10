import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
import rospy
import numpy as np

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

dist_coeffs = np.array([0, 0, 0, 0], dtype=float)  # Example values, replace if needed

# ROS Publishers
pose_array_pub = None
processed_image_pub = None
depth_image = None  # Latest depth image


def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logerr(f"Error converting depth image: {e}")


def process_image(msg):
    global bridge, aruco_dict, parameters, pose_array_pub, processed_image_pub, depth_image

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
        pose_array = PoseArray()  # Store poses of all detected markers
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = "camera_frame"

        for i, marker_id in enumerate(ids.flatten()):
            # Compute the center of the marker in pixel coordinates
            marker_corners = corners[i][0]
            pixel_x = int(marker_corners[:, 0].mean())  # Mean of x-coordinates
            pixel_y = int(marker_corners[:, 1].mean())  # Mean of y-coordinates

            # Get the depth value at the marker's center
            try:
                Z = depth_image[pixel_y, pixel_x] / 1000.0  # Depth in meters (convert from mm)
            except IndexError:
                rospy.logerr("Pixel coordinates out of depth image bounds")
                continue

            if Z == 0:
                rospy.logwarn("Invalid depth value (0), skipping marker")
                continue

            # Calculate real-world X and Y
            X = (pixel_x - cx) * Z / fx
            Y = (pixel_y - cy) * Z / fy

            # Log the calculated position
            rospy.loginfo(f"Marker ID {marker_id}: X={X:.2f} m, Y={Y:.2f} m, Z={Z:.2f} m")

            # Publish the marker's pose
            pose_msg = Pose()
            pose_msg.position.x = X
            pose_msg.position.y = Y
            pose_msg.position.z = Z
            pose_array.poses.append(pose_msg)

            # Draw the marker and pose information on the image
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.circle(frame, (pixel_x, pixel_y), 5, (0, 255, 0), -1)  # Highlight the center
            cv2.putText(frame, f"ID:{marker_id} X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}",
                        (pixel_x + 10, pixel_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Publish all poses as a PoseArray
        pose_array_pub.publish(pose_array)

    else:
        rospy.loginfo("No markers detected or depth image not available.")

    # Publish the processed image
    try:
        processed_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        processed_image_pub.publish(processed_image)
    except Exception as e:
        rospy.logerr(f"Error publishing processed image: {e}")


def main():
    global pose_array_pub, processed_image_pub

    # Initialize the ROS node
    rospy.init_node("aruco_multiple_pose_with_depth_and_image", anonymous=True)

    # Subscribe to RGB and Depth topics
    camera_topic = "/camera/color/image_raw"  # Replace with your RGB image topic
    depth_topic = "/camera/depth/image_raw"  # Replace with your depth topic
    rospy.Subscriber(camera_topic, Image, process_image)
    rospy.Subscriber(depth_topic, Image, depth_callback)

    # Publishers for marker poses and processed image
    pose_array_pub = rospy.Publisher("/aruco_marker_poses", PoseArray, queue_size=10)
    processed_image_pub = rospy.Publisher("/aruco_processed_image", Image, queue_size=10)

    rospy.loginfo("Aruco Multiple Pose and Image Publisher Node Started")

    # Keep the node running
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

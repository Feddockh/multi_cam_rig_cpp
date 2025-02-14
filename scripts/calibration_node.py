#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import message_filters

class CameraCalibrationNode(Node):
    def __init__(self):
        super().__init__('camera_calibration_node')
        self.declare_parameter("director_topic", "/multi_cam_rig/director")
        self.declare_parameter('left_image_topic', '/flir_node/firefly_left/image_raw')
        self.declare_parameter('right_image_topic', '/flir_node/firefly_right/image_raw')
        self.declare_parameter('trigger_frequency', 1)  # Hz

        # Calibration settings:
        self.checkerboard_dims = (7, 6)    # internal corners per row/column (adjust as needed)
        self.square_size = 0.024           # size of one square (meters)
        self.required_pairs = 15           # number of good pairs to collect
        self.epi_threshold = 2.0           # desired RMS or epipolar error threshold

        # Prepare object points for the checkerboard (like (0,0,0), (1,0,0), …)
        objp = np.zeros((self.checkerboard_dims[0]*self.checkerboard_dims[1], 3), np.float32)
        objp[:,:2] = np.mgrid[0:self.checkerboard_dims[0],
                               0:self.checkerboard_dims[1]].T.reshape(-1,2)
        objp = objp * self.square_size

        self.objp = objp
        self.objpoints = []         # 3d points in real world space
        self.imgpoints_left = []      # 2d points in left image
        self.imgpoints_right = []     # 2d points in right image

        # Flag to indicate calibration has been done
        self.calibrated = False

        # Retrieve parameters
        director_topic = self.get_parameter("director_topic").value
        left_topic = self.get_parameter('left_image_topic').value
        right_topic = self.get_parameter('right_image_topic').value
        trigger_frequency = self.get_parameter('trigger_frequency').value

        # Set up director publisher (could be used to signal capture events)
        self.director_pub = self.create_publisher(String, director_topic, 10)

        self.get_logger().info(f"Subscribing to left topic: {left_topic}")
        self.get_logger().info(f"Subscribing to right topic: {right_topic}")
        self.get_logger().info(f"Publishing capture triggers to: {director_topic}")
        self.get_logger().info(f"Trigger frequency set to: {trigger_frequency} Hz")

        self.bridge = CvBridge()

        # Create message_filters subscribers for both image topics
        self.left_sub = message_filters.Subscriber(self, Image, left_topic)
        self.right_sub = message_filters.Subscriber(self, Image, right_topic)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=10,
            slop=1.0
        )
        self.ts.registerCallback(self.image_callback)

        trigger_interval = 1 / trigger_frequency
        self.timer = self.create_timer(trigger_interval, self.trigger)

        self.capture_number = 0

        # Initialize OpenCV window (resizable)
        cv2.namedWindow("Stereo Calibration View", cv2.WINDOW_NORMAL)

    def trigger(self):
        """Periodically send a trigger message (if needed) to your director."""
        msg = String()
        msg.data = "Capture " + str(self.capture_number)
        self.director_pub.publish(msg)
        self.get_logger().info("Sent trigger message: " + msg.data)
        self.capture_number += 1

    def image_callback(self, left_msg, right_msg):
        if self.calibrated:
            # Optionally, once calibrated, you can stop processing further images.
            return

        self.get_logger().info("Received synchronized image pair.")

        try:
            left_image = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding='bgr8')
            right_image = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        # Resize images if needed (optional, for display)
        if left_image.shape[0] != right_image.shape[0]:
            height = left_image.shape[0]
            scale_factor = height / right_image.shape[0]
            width = int(right_image.shape[1] * scale_factor)
            right_image = cv2.resize(right_image, (width, height))

        # Combine images side by side for display
        combined_image = np.hstack((left_image, right_image))
        # Optionally, scale down for display:
        max_width = 1280
        max_height = 720
        h, w, _ = combined_image.shape
        if w > max_width or h > max_height:
            scale_factor = min(max_width/w, max_height/h)
            new_width = int(w*scale_factor)
            new_height = int(h*scale_factor)
            combined_image = cv2.resize(combined_image, (new_width, new_height), interpolation=cv2.INTER_AREA)
        cv2.imshow("Stereo Calibration View", combined_image)
        cv2.waitKey(10)

        # Now, perform checkerboard detection on both images:
        gray_left = cv2.cvtColor(left_image, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_image, cv2.COLOR_BGR2GRAY)

        ret_left, corners_left = cv2.findChessboardCorners(gray_left, self.checkerboard_dims, None)
        ret_right, corners_right = cv2.findChessboardCorners(gray_right, self.checkerboard_dims, None)

        if ret_left and ret_right:
            # Refine corners to subpixel accuracy:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_left = cv2.cornerSubPix(gray_left, corners_left, (11,11), (-1,-1), criteria)
            corners_right = cv2.cornerSubPix(gray_right, corners_right, (11,11), (-1,-1), criteria)

            # Draw and display the corners (optional)
            cv2.drawChessboardCorners(left_image, self.checkerboard_dims, corners_left, ret_left)
            cv2.drawChessboardCorners(right_image, self.checkerboard_dims, corners_right, ret_right)

            # Append the object points and image points
            self.objpoints.append(self.objp)
            self.imgpoints_left.append(corners_left)
            self.imgpoints_right.append(corners_right)
            self.get_logger().info(f"Calibration pair collected: {len(self.objpoints)} / {self.required_pairs}")

            # Optionally, you could also display the images with drawn corners here.

            # If we have enough pairs, attempt stereo calibration:
            if len(self.objpoints) >= self.required_pairs:
                self.calibrate_cameras(gray_left.shape[::-1])  # image size as (width, height)

    def calibrate_cameras(self, image_size):
        self.get_logger().info("Starting stereo calibration...")

        # First, calibrate each camera individually.
        ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_left, image_size, None, None)
        ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
            self.objpoints, self.imgpoints_right, image_size, None, None)

        # Now, perform stereo calibration:
        flags = cv2.CALIB_FIX_INTRINSIC
        criteria = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        ret, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T, E, F = cv2.stereoCalibrate(
            self.objpoints, self.imgpoints_left, self.imgpoints_right,
            mtx_left, dist_left, mtx_right, dist_right, image_size,
            criteria=criteria, flags=flags)

        self.get_logger().info(f"Stereo Calibration RMS error: {ret:.4f}")

        # Check if RMS (or epi) is under the threshold
        if ret < self.epi_threshold:
            self.get_logger().info("Calibration successful! Epipolar error is under threshold.")
            self.calibrated = True
            # Optionally, save the calibration parameters to file or publish them.
            self.save_calibration(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, R, T)
        else:
            self.get_logger().warn("Calibration RMS error is too high. Continue capturing...")

    def save_calibration(self, mtx_left, dist_left, mtx_right, dist_right, R, T):
        # Save calibration parameters to a file (or do something else with them)
        np.savez("stereo_calibration_parameters.npz",
                 mtx_left=mtx_left, dist_left=dist_left,
                 mtx_right=mtx_right, dist_right=dist_right,
                 R=R, T=T)
        self.get_logger().info("Calibration parameters saved to stereo_calibration_parameters.npz")

def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down camera calibration node.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

"""
director_time_synchronizer.py
Author: Hayden Feddock
Date: 1/14/2025

This script listens for director commands and camera images, and saves images to disk
when a command is received. The images are saved with the timestamp of the director command.
"""

import os
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from std_msgs.msg import String
from sensor_msgs.msg import Image


class DirectorRequest:
    def __init__(self, id: int, timestamp: Time, validity_s: float, camera_names: list) -> None:
        """
        Holds data for a single 'director command' request.
        """
        self.id = id
        self.request_timestamp = timestamp
        self.request_duration = Duration(seconds=validity_s)
        self.deadline = self.request_timestamp + self.request_duration
        self.camera_names = camera_names
        self.images_received = dict()  # camera_name -> cv_image

    def is_expired(self, current_time: Time) -> bool:
        """
        Check if we've passed the deadline.
        """
        return current_time > self.deadline

    def add_image(self, camera_name: str, timestamp: Time, cv_image: cv2.Mat) -> bool:
        """
        Store an image if it's >= request_timestamp and we still haven't stored an image 
        for this camera.
        """
        if camera_name not in self.images_received:
            if timestamp >= self.request_timestamp and timestamp <= self.deadline:
                self.images_received[camera_name] = cv_image
                return True
        return False

    def is_fulfilled(self) -> bool:
        """
        True if we have an image for each camera.
        """
        return len(self.images_received) == len(self.camera_names)


class DirectorTimeSynchronizer(Node):
    def __init__(self) -> None:
        """
        Controls the synchronization of camera images with director commands.
        """
        super().__init__('director_time_synchronizer')

        # Define parameters
        self.declare_parameter('validity_window', 1.0)
        self.declare_parameter('director_topic', '/multi_cam_rig/director')
        self.declare_parameter('camera_names', [
            'firefly_left',
            'firefly_right',
            'ximea',
            'zed_left',
            'zed_right'
        ])
        self.declare_parameter('camera_topics', [
            '/flir_node/firefly_left/image_raw',
            '/flir_node/firefly_right/image_raw',
            '/multi_cam_rig/ximea/image',
            '/multi_cam_rig/zed/left_image',
            '/multi_cam_rig/zed/right_image'
        ])
        self.declare_parameter('output_dir', '~/data/director_sync')

        # Set up parameters
        self.validity_window = self.get_parameter('validity_window').get_parameter_value().double_value
        self.director_topic = self.get_parameter('director_topic').get_parameter_value().string_value
        self.camera_names = self.get_parameter('camera_names').get_parameter_value().string_array_value
        self.camera_topics = self.get_parameter('camera_topics').get_parameter_value().string_array_value
        raw_output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.output_dir = os.path.expanduser(raw_output_dir)
        os.makedirs(self.output_dir, exist_ok=True)

        # Map topics to camera names
        self.topic_camera_map = {topic: name for topic, name in zip(self.camera_topics, self.camera_names)}

        # request queue from the director (arrivals are in order of timestamps so no need to sort)
        self.active_request: DirectorRequest = None

        # Director command subscription
        self.director_sub = self.create_subscription(
            String,
            self.director_topic,
            self.director_callback,
            10
        )

        # Create cv bridge
        self.bridge = CvBridge()

        # Image subscriptions
        for topic in self.camera_topics:
            self.create_subscription(Image, topic, lambda msg, t=topic: self.image_callback(t, msg), 10)

        self.get_logger().info("director_time_synchronizer initialized.")

    def director_callback(self, msg: String) -> None:
        """
        A new command arrives from the director. If it's a 'capture' command, we'll open a new request.
        """

        # Get the text of the command
        command = msg.data

        # Get the timestamp of the command
        timestamp = self.get_clock().now()

        # If the command is 'capture #', open a new request
        if command.startswith('capture'):

            # Get the id number from the command
            id = int(command.split()[1])

            # Destroy the previous request if it exists
            if self.active_request is not None:
                self.get_logger().info(f"Request {self.active_request.id} expired.")
            
            # Create a new request object
            self.active_request = DirectorRequest(
                id=id,
                timestamp=timestamp,
                validity_s=self.validity_window,
                camera_names=self.camera_names
            )
            self.get_logger().info(f"New request opened with deadline at {self.active_request.deadline.to_msg().sec}")

    def image_callback(self, camera_topic: str, msg: Image) -> bool:
        """
        Each camera publishes an Image. We'll store it in the active request if it's valid.
        """

        # Get the camera name from the topic
        camera_name = self.topic_camera_map[camera_topic]

        # Get the timestamp of the image
        timestamp = self.get_clock().now()

        # Convert Image to CV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Attempt to add the image to the request
        if self.active_request is None:
            self.get_logger().info(f"No active request for {camera_name} at {timestamp.to_msg().sec}")
            return False
        
        image_added = self.active_request.add_image(
            camera_name=camera_name,
            timestamp=timestamp,
            cv_image=cv_image
        )

        # If the request is fulfilled, save the images to disk
        if image_added:
            self.get_logger().info(f"Added image for {camera_name} at {timestamp.to_msg().sec}")
            if self.active_request.is_fulfilled():
                self.save_request_images(self.active_request)
                self.active_request = None
                self.get_logger().info("Request fulfilled.")
        else:
            self.get_logger().info(f"Image for {camera_name} at {timestamp.to_msg().sec} not added")

        return image_added
        
    def save_request_images(self, request: DirectorRequest):
        """
        Save each camera's image to disk, using the director's command timestamp
        for the filename.
        """

        # Use the time stamp of the director command for the filename
        director_sec = request.request_timestamp.nanoseconds // 1_000_000_000
        director_nsec = request.request_timestamp.nanoseconds % 1_000_000_000
        stamp_str = f"{director_sec}_{director_nsec}"

        # Save each image
        for cam_name, cv_img in request.images_received.items():
            filename = f"{stamp_str}_{cam_name}.jpg"
            out_path = os.path.join(self.output_dir, filename)
            cv2.imwrite(out_path, cv_img)

        self.get_logger().info(f"Images saved to {self.output_dir}")

def main(args=None):
    rclpy.init(args=args)
    node = DirectorTimeSynchronizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

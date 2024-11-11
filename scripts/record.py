#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import rospkg
import os
import subprocess
import signal
import sys
import datetime

class RecordNode(Node):
    def __init__(self):
        super().__init__('record_node')

        # Declare parameters for bag type and folder locations
        self.bag_type = self.declare_parameter('bag_type', 'compressed').value
        self.raw_bag_folder = self.declare_parameter('raw_bag_folder', self.get_default_bag_folder('raw')).value
        self.compressed_bag_folder = self.declare_parameter('compressed_bag_folder', self.get_default_bag_folder('compressed')).value
        self.undistorted_bag_folder = self.declare_parameter('undistorted_bag_folder', self.get_default_bag_folder('undistorted')).value

        # Get the current timestamp for the filename
        self.time = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')

        # Determine the appropriate filename and command based on the 'bag_type'
        if self.bag_type == 'raw':
            self.filename = f"{self.raw_bag_folder}/{self.time}_raw.bag"
            self.command = f"ros2 bag record -o {self.filename} /insta_image_yuv"
            self.record_location = self.raw_bag_folder
        elif self.bag_type == 'compressed':
            self.filename = f"{self.compressed_bag_folder}/{self.time}_compressed.bag"
            self.command = f"ros2 bag record -o {self.filename} /back_camera_image/compressed /front_camera_image/compressed"
            self.record_location = self.compressed_bag_folder
        elif self.bag_type == 'undistorted':
            self.filename = f"{self.undistorted_bag_folder}/{self.time}_undistorted.bag"
            self.command = f"ros2 bag record -o {self.filename} /back_camera_image/compressed /front_camera_image/compressed"
            self.record_location = self.undistorted_bag_folder
        else:
            self.get_logger().error('Invalid bag_type parameter.')
            sys.exit(1)

        self.get_logger().warn(f"Recording to: {self.filename}")

        # Start the recording process
        self.process = subprocess.Popen(self.command, shell=True)

        # Register the signal handler for SIGINT (Ctrl + C)
        signal.signal(signal.SIGINT, self.signal_handler)

    def signal_handler(self, sig, frame):
        self.get_logger().warn('Stopping recording...')
        self.process.terminate()
        self.process.wait()
        self.get_logger().warn('Recording stopped.')
        rclpy.shutdown()

    def get_default_bag_folder(self, bag_type):
        default_dir = rospkg.RosPack().get_path('insta360_ros2_driver')
        default_dir = os.path.join(default_dir, 'bag')
        return os.path.join(default_dir, bag_type)


def main(args=None):
    rclpy.init(args=args)
    record_node = RecordNode()

    try:
        rclpy.spin(record_node)
    except KeyboardInterrupt:
        pass
    finally:
        record_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

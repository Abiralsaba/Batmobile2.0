#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import CompressedImage
import os
import sys


class WebcamStreamNode(Node):
    def __init__(self):
        super().__init__('Science_Bifrost_View_Node')

        self.publisher = self.create_publisher(
            CompressedImage, '/science/camera/compressed', 10)

        self.timer = self.create_timer(0.05, self.publish_webcam_image)

        self.camera_model = 'ROG_EYE_S'
        self.cap = cv2.VideoCapture()
        self.cap.open(self.find_camera_interface(self.camera_model))

    def publish_webcam_image(self):
        ret, frame = self.cap.read()
        if ret:

            # Fix mirrored image (flip left-right)
            # frame = cv2.flip(frame, 1)

            # frame = cv2.resize(frame, (320, 240))
            frame = cv2.resize(frame, (1280, 720))

            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            _, compressed_data = cv2.imencode('.jpg', frame, encode_param)

            msg = CompressedImage()
            msg.format = 'jpeg'
            msg.data = compressed_data.tobytes()

            self.publisher.publish(msg)
            self.get_logger().info('Frame Published')

    def find_camera_interface(self, key):
        folder_path = '/dev/v4l/by-id/'
        interface = None

        if os.path.exists(folder_path):
            file_names = os.listdir(folder_path)
            for file_name in file_names:
                if key in file_name:
                    interface = folder_path + file_name
                    interface = list(interface)
                    interface[-1] = '0'
                    interface = ''.join(interface).strip()

                    self.get_logger().info(
                        f'Camera interface: {interface}')
                    break

        if interface is None:
            self.get_logger().info('Camera interface Not Found Exiting')
            sys.exit(0)

        return interface


def main(args=None):
    rclpy.init(args=args)
    node = WebcamStreamNode()
    rclpy.spin(node)

    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
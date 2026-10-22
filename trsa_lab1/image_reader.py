"""
Read images from /camera/image_raw and display them in a window.

Window closes automatically if no new frames arrive for 1 second, 
but the node continues to run and will reopen the window if new frames arrive.
"""

import cv2
import time
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.executors import ExternalShutdownException


TIMEOUT = 1.0               # seconds without frames before closing

class ImageReader(Node):
    def __init__(self):
        super().__init__('image_reader')
        self.br = CvBridge()
        self.last_raw_frame = None
        self.window_raw_open = False
        self.create_timer(0.2, self.watchdog_raw)

        self.create_subscription(Image, '/camera/image_raw', self.raw_cb, 10)
        self.create_subscription(Image, '/camera/image_processed', self.proc_cb, 10)

    def raw_cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('image_reader_raw', frame)
        cv2.waitKey(1)

        self.window_raw_open = True
        self.last_raw_frame = time.monotonic()

    def proc_cb(self, msg):
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='mono8')
        cv2.imshow('image_reader_processed', frame)
        cv2.waitKey(1)

    # Watchdog functions to close windows if no new raw frames arrive for TIMEOUT seconds
    def watchdog_raw(self):
        if (self.window_raw_open and self.last_raw_frame is not None
                and time.monotonic() - self.last_raw_frame > TIMEOUT):
            self.get_logger().info('Raw stream ended. Closing window.')
            cv2.destroyAllWindows()
            cv2.waitKey(1)
            self.window_raw_open = False

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageReader()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
            cv2.destroyAllWindows()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
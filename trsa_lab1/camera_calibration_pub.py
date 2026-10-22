"""
Replay the TRSA calibration images on /camera/image_raw until Ctrl+C.

Default image location: <trsa_lab1 share directory>/calibration/images.
The images must have numeric filenames, for example 0.jpeg, 1.jpeg, ... .
Startup parameters: images_dir, publish_rate (Hz), and frame_id.
"""

import cv2
import math
import rclpy

from pathlib import Path
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory


class CameraCalibrationPub(Node):
    def __init__(self):
        super().__init__('camera_calibration_pub_node')

        default_dir = (
            Path(get_package_share_directory('trsa_lab1'))
            / 'calibration' / 'images'
        )
        self.declare_parameter('images_dir', str(default_dir))
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('frame_id', 'camera_optical_frame')

        images_dir = Path(self.get_parameter('images_dir').value).expanduser()
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value

        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError('publish_rate must be a finite number greater than zero.')
        if not self.frame_id or self.frame_id.startswith('/'):
            raise ValueError('frame_id must be nonempty and must not start with "/".')
        if not images_dir.is_dir():
            raise RuntimeError(
                f'Calibration image directory not found: {images_dir}. '
                'Check the calibration/images entry in setup.py, rebuild the '
                'package, and source the workspace.'
            )

        # Numeric ordering avoids the sequence 0, 1, 10, 100, ... .
        # Ignoring nonnumeric filenames also excludes macOS metadata files.
        self.image_paths = sorted(
            (path for path in images_dir.glob('*.jpeg')
             if path.is_file() and path.stem.isdecimal()),
            key=lambda path: int(path.stem),
        )
        if not self.image_paths:
            raise RuntimeError(f'No numbered .jpeg images found in: {images_dir}')

        # Validate the complete dataset before starting the stream.
        self.image_shape = None
        for path in self.image_paths:
            frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if frame is None:
                raise RuntimeError(f'Cannot read calibration image: {path}')
            if self.image_shape is None:
                self.image_shape = frame.shape
            elif frame.shape != self.image_shape:
                raise RuntimeError(
                    f'Image dimensions do not match the other images: {path}'
                )

        self.br = CvBridge()
        self.img_idx = 0
        self.image_pub_raw = self.create_publisher(Image, '/camera/image_raw', 1)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_image)

        self.get_logger().info(
            f'Publishing {len(self.image_paths)} calibration images at '
            f'{publish_rate:g} Hz from {images_dir}. '
            'The sequence repeats automatically. Keep this node running while '
            'clicking CALIBRATE and SAVE; stop it afterwards with Ctrl+C.'
        )

    def publish_image(self) -> None:
        image_path = self.image_paths[self.img_idx]
        frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)

        # Also handle a file being removed or changed after startup.
        if frame is None or frame.shape != self.image_shape:
            self.get_logger().error(
                f'Image became unreadable or changed dimensions: {image_path}. '
                'Publishing stopped. Restore the dataset and restart this node.'
            )
            self.timer.cancel()
            return

        ros_img_raw = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_img_raw.header.frame_id = self.frame_id
        ros_img_raw.header.stamp = self.get_clock().now().to_msg()
        self.image_pub_raw.publish(ros_img_raw)

        # Keep publishing after the final image, including during calibration.
        self.img_idx = (self.img_idx + 1) % len(self.image_paths)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraCalibrationPub()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

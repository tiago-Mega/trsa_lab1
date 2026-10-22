import os
import cv2
import yaml
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory

class CameraRectifier(Node):
    def __init__(self):
        super().__init__('camera_rectifier')
        self.br = CvBridge()
        self.image_pub_rect = self.create_publisher(Image, '/camera/image_rect', 10)

        self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # Load camera calibration parameters from YAML file
        package_share_directory = get_package_share_directory('trsa_lab1')
        calibration_file_path = os.path.join(package_share_directory, 'calibration', 'ost.yaml')

        self.load_camera_calibration(calibration_file_path)
        

    def load_camera_calibration(self, calibration_file_path):

        with open(calibration_file_path, 'r') as file:
            self.camera_info = CameraInfo()
            calibration_data = yaml.safe_load(file)

            self.camera_info.width = calibration_data['image_width']
            self.camera_info.height = calibration_data['image_height']
            self.camera_info.k = calibration_data['camera_matrix']['data']
            self.camera_info.p = calibration_data['projection_matrix']['data']  
            self.camera_info.r = calibration_data['rectification_matrix']['data']
            self.camera_info.d = calibration_data['distortion_coefficients']['data']
            self.camera_info.distortion_model = calibration_data['distortion_model']

        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        rectified_frame = frame.copy()

        # Rectify the image using the camera model
        self.camera_model.rectifyImage(frame, rectified_frame) 

        # Convert rectified OpenCV image back to ROS Image message
        rectified_msg = self.br.cv2_to_imgmsg(rectified_frame, encoding='bgr8')
        rectified_msg.header.stamp = self.get_clock().now().to_msg()
        rectified_msg.header.frame_id = "camera"
        
        # Publish the rectified image and camera info
        self.image_pub_rect.publish(rectified_msg)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraRectifier()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
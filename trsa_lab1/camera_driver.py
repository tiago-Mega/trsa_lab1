import os
import cv2
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory

class CameraDriver(Node):
    def __init__(self):
        super().__init__('camera_driver')
        self.br = CvBridge()
        self.declare_parameter('loop', True)
        self.loop = self.get_parameter('loop').value
        self.timer = self.create_timer(0.03, self.timer_callback)                           

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        # Load video file from package share directory
        package_share_directory = get_package_share_directory('trsa_lab1')
        
        video_file_path = os.path.join(package_share_directory, 'video', 'test.mov')
        video_detection_file_path = os.path.join(package_share_directory, 'video', 'test_obj.mp4')

        self.cap = cv2.VideoCapture(str(video_file_path))     # Open the video file
        
        if not self.cap.isOpened():
            raise RuntimeError('Could not open video file')

        self.finished = False
    
    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret:
            if not self.loop:
                self.get_logger().info('End of video. Shutting down.')
                self.finished = True
                self.timer.cancel()
                return
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            ret, frame = self.cap.read()
        
        ros_image = self.br.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = "camera"
        self.publisher.publish(ros_image)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraDriver()
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.cap.release()   
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
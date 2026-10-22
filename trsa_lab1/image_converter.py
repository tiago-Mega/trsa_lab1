import cv2
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.executors import ExternalShutdownException

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')
        self.br = CvBridge()

        self.image_pub = self.create_publisher(Image, '/camera/image_processed', 10)

        self.subscription = self.create_subscription(Image, '/camera/image_rect', self.image_callback, 10)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return

        # Process the image (e.g., convert to grayscale)
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Change the Gaussian blur kernel size to (5, 5) for better edge detection
        #processed_frame = cv2.GaussianBlur(gray_frame, (21, 21), 0)
        processed_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
        processed_frame = cv2.Canny(processed_frame, 60, 150)

        # Convert processed OpenCV image back to ROS Image message
        processed_msg = self.br.cv2_to_imgmsg(processed_frame, encoding='mono8')
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        processed_msg.header.frame_id = "camera"

        # Publish the processed image
        self.image_pub.publish(processed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageConverter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
import cv2
import rclpy

import numpy as np

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from rclpy.executors import ExternalShutdownException


class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.bridge = CvBridge()

        # Declare parameters for target color, shape, and tolerances
        self.declare_parameter('target_color', 'blue')
        self.declare_parameter('target_shape', 'rectangle')
        self.declare_parameter('tol_x', 0.05)  # 5% horizontal tolerance
        self.declare_parameter('tol_y', 0.05)  # 5% vertical tolerance
        self.declare_parameter('min_contour_area', 1200.0)

        # Subscribe to rectified or raw image topic
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',  # Change to '/camera/image_rect' if using rectified images
            self.image_callback,
            10
        )

        self.image_pub = self.create_publisher(Image, '/object_detection/image', 10)
        self.count_pub = self.create_publisher(Int32, '/object_detection/count', 10)
        self.alignment_pub = self.create_publisher(String, '/object_detection/alignment', 10)

        self.get_logger().info('Object Detection node has started.')

    def segment_color(self, hsv_img, color_name):
        """Segment image by HSV ranges with dual-band support for red."""
        if color_name == 'blue':
            lower = np.array([95, 100, 50], dtype=np.uint8)
            upper = np.array([135, 255, 255], dtype=np.uint8)
            return cv2.inRange(hsv_img, lower, upper)

        elif color_name == 'green':
            lower = np.array([35, 70, 40], dtype=np.uint8)
            upper = np.array([85, 255, 255], dtype=np.uint8)
            return cv2.inRange(hsv_img, lower, upper)

        elif color_name == 'red':
            # Red wraps across the 0/180 hue threshold
            lower1, upper1 = np.array([0, 90, 50]), np.array([10, 255, 255])
            lower2, upper2 = np.array([170, 90, 50]), np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv_img, lower1, upper1)
            mask2 = cv2.inRange(hsv_img, lower2, upper2)
            return cv2.bitwise_or(mask1, mask2)

        return np.zeros(hsv_img.shape[:2], dtype=np.uint8)

    def classify_shape(self, contour):
        """Identify shape using polygon approximation and circularity."""
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 'unknown'

        # Polygon vertex approximation
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        num_vertices = len(approx)

        area = cv2.contourArea(contour)
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)

        if num_vertices == 3:
            return 'triangle'
        elif num_vertices == 4:
            # Check rectangularity against oriented bounding box
            rect = cv2.minAreaRect(contour)
            box_area = rect[1][0] * rect[1][1]
            if box_area > 0 and (area / box_area) > 0.75:
                return 'rectangle'
            return 'quadrilateral'
        elif circularity > 0.75:
            return 'circle'

        return 'unknown'

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion error: {e}')
            return

        annotated_img = cv_img.copy()
        h, w = cv_img.shape[:2]
        center_x, center_y = w // 2, h // 2

        target_color = self.get_parameter('target_color').value
        target_shape = self.get_parameter('target_shape').value
        tol_x_pct = self.get_parameter('tol_x').value
        tol_y_pct = self.get_parameter('tol_y').value
        min_area = self.get_parameter('min_contour_area').value

        tol_px_x = int(w * tol_x_pct)
        tol_px_y = int(h * tol_y_pct)

        # Preprocessing: blur and convert to HSV
        blurred = cv2.GaussianBlur(cv_img, (7, 7), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        color_mask = self.segment_color(hsv, target_color)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cleaned_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, kernel)
        cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        valid_targets = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area:
                continue

            detected_shape = self.classify_shape(cnt)
            if detected_shape == target_shape:
                # Calculate centroid
                M = cv2.moments(cnt)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])
                    valid_targets.append({
                        'contour': cnt,
                        'center': (cx, cy),
                        'area': area
                    })

        # Draw alignment region target box
        cv2.rectangle(
            annotated_img,
            (center_x - tol_px_x, center_y - tol_px_y),
            (center_x + tol_px_x, center_y + tol_px_y),
            (255, 255, 0),
            2
        )
        cv2.drawMarker(annotated_img, (center_x, center_y), (255, 255, 0), cv2.MARKER_CROSS, 14, 2)

        # Draw and annotate valid targets
        for target in valid_targets:
            cnt = target['contour']
            cx, cy = target['center']

            # Rotated bounding box
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = box.astype(int)
            cv2.drawContours(annotated_img, [box], 0, (0, 255, 0), 2)

            # Center marker
            cv2.circle(annotated_img, (cx, cy), 5, (0, 0, 255), -1)

            label = f'{target_color} {target_shape}'
            cv2.putText(annotated_img, label, (cx - 40, cy - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        # Bonus: Visual Alignment Logic
        alignment_state = 'NO_TARGET'
        if valid_targets:
            # Select target with largest area
            primary = max(valid_targets, key=lambda t: t['area'])
            cx, cy = primary['center']

            # Highlight selected primary alignment target
            cv2.circle(annotated_img, (cx, cy), 8, (255, 0, 255), -1)

            dx = cx - center_x
            dy = cy - center_y

            norm_dx = abs(dx) / float(tol_px_x) if tol_px_x > 0 else 0.0
            norm_dy = abs(dy) / float(tol_px_y) if tol_px_y > 0 else 0.0

            within_x = abs(dx) <= tol_px_x
            within_y = abs(dy) <= tol_px_y

            if within_x and within_y:
                alignment_state = 'ALIGNED'
            elif norm_dx >= norm_dy:
                alignment_state = 'RIGHT' if dx > 0 else 'LEFT'
            else:
                alignment_state = 'BELOW' if dy > 0 else 'ABOVE'

            # Display coordinate offsets
            offset_text = f'Offset: dx={dx}, dy={dy}'
            cv2.putText(annotated_img, offset_text, (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

        # Draw HUD overlays
        count_val = len(valid_targets)
        cv2.putText(annotated_img, f'Targets Detected: {count_val}', (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(annotated_img, f'Alignment: {alignment_state}', (20, 105),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 100, 0), 2)

        # Publish results
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_img, encoding='bgr8'))
        
        count_msg = Int32()
        count_msg.data = count_val
        self.count_pub.publish(count_msg)

        align_msg = String()
        align_msg.data = alignment_state
        self.alignment_pub.publish(align_msg)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObjectDetection()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
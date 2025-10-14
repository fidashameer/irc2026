import os
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time


class Detection:
    def __init__(self, class_name, bbox_area, bbox_coords):
        self.class_name = class_name
        self.bbox_area = bbox_area
        self.bbox_coords = bbox_coords


class YOLOModel:
    def __init__(self, model_path, input_size=(640, 480)):
        self.model = YOLO(model_path)
        self.bridge = CvBridge()
        self.input_size = input_size

    def detect_image(self, image_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            cv_image_resized = cv2.resize(cv_image, self.input_size)
            results = self.model(cv_image_resized)
            detections = []
            for result in results:
                for box in result.boxes:
                    class_name = result.names[int(box.cls[0])]
                    x1, y1, x2, y2 = box.xyxy[0]
                    bbox_area = (x2 - x1) * (y2 - y1)
                    detections.append(Detection(class_name, bbox_area, (x1, y1, x2, y2)))
            return detections, cv_image, cv_image_resized.shape[:2]
        except Exception as e:
            print(f"[YOLOModel] Detection failed: {e}")
            return [], None, (1, 1)


class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection_node')

        # Declare and get parameters
        self.declare_parameter('model_path', '')  # fallback to packaged model if empty or 'best.pt'
        self.declare_parameter('camera_topic', '/image_raw')
        self.declare_parameter('visualize', True)

        input_model_path = self.get_parameter('model_path').value
        camera_topic = self.get_parameter('camera_topic').value
        self.visualize = self.get_parameter('visualize').value

        if input_model_path == '' or input_model_path == 'best.pt':
            model_path = os.path.join(get_package_share_directory('my_rob_con'), 'rado', 'best.pt')
        else:
            model_path = input_model_path

        self.get_logger().info(f'Using YOLO model: {model_path}')
        self.yolo_model = YOLOModel(model_path)

        # Publishers matching movement_node expectations
        self.cone_pub = self.create_publisher(Bool, 'cone_detected', 10)
        self.coverage_pub = self.create_publisher(Float32, 'cone_coverage', 10)
        self.centroid_pub = self.create_publisher(Float32, 'cone_position', 10)  # Float32 for normalized x

        qos = QoSProfile(depth=10)
        self.subscription = self.create_subscription(Image, camera_topic, self.image_callback, qos)

        self.get_logger().info(f'Cone Detection Node subscribed to: {camera_topic}')
        self.last_log_time = time.time()

    def image_callback(self, msg):
        detections, cv_image, (h, w) = self.yolo_model.detect_image(msg)
        if cv_image is None:
            return

        cones = [d for d in detections if 'cone' in d.class_name.lower()]

        cone_found = len(cones) > 0
        coverage = 0.0
        cone_x = 0.5  # default center

        if cone_found:
            largest = max(cones, key=lambda d: d.bbox_area)
            x1, y1, x2, y2 = largest.bbox_coords
            coverage = largest.bbox_area / (w * h) * 100.0
            cone_x = float((x1 + x2) / 2 / w)

            if self.visualize:
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.putText(cv_image, f"Cone {coverage:.1f}%", (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.cone_pub.publish(Bool(data=cone_found))
        self.coverage_pub.publish(Float32(data=coverage))
        self.centroid_pub.publish(Float32(data=cone_x))

        if time.time() - self.last_log_time > 2:
            self.get_logger().info(f'Cone detected: {cone_found}, Coverage: {coverage:.2f}%, X position: {cone_x:.3f}')
            self.last_log_time = time.time()

        if self.visualize:
            cv2.imshow("YOLO Cone Detection", cv_image)
            cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Cone Detection Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import cvzone
import math
from cv_bridge import CvBridge
from ultralytics import YOLO

class object_detector(Node):
    def __init__(self):
        super().__init__('object_detector')

        
        self.publisher_ = self.create_publisher(Float32MultiArray, 'object_position', 10)

       
        # self.subscription_ = self.create_subscription(String, 'motion_status', self.motion_callback, 10)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
        # self.model = YOLO('../../../../../Vision_system/mymodel/runs/detect/train4/weights/best.pt')
        # Inside object_detector.py
        self.model = YOLO('/home/samiul/Thesis_ws/System/Vision_system/mymodel/runs/detect/train4/weights/best.pt')

        self.names = self.model.names

        self.timer = self.create_timer(0.05, self.detect_objects)  

        self.get_logger().info("Detection Node Initialized. Starting object detection...")

    def detect_objects(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().warn("Failed to capture frame from camera.")
            return

        results = self.model(img, stream=True)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w, h = x2 - x1, y2 - y1

                cvzone.cornerRect(img, (x1, y1, w, h), l=9)

                conf = round(float(box.conf[0]), 2)
                cls_id = int(box.cls[0])
                class_name = self.names[cls_id]

               
                center_x = x1 + w / 2
                center_y = y1 + h / 2

                msg = Float32MultiArray()
                msg.data = [float(center_x), float(center_y), float(cls_id), float(conf)]
                self.publisher_.publish(msg)

                self.get_logger().info(f"Published object: {class_name} @ ({center_x:.1f}, {center_y:.1f}), conf={conf}")

               
                cvzone.putTextRect(img, f"{class_name} {conf}", (max(0, x1), max(35, y1)), scale=1, thickness=1)

        cv2.imshow('Detection View', img)
        cv2.waitKey(1)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = object_detector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Detection Node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

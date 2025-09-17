#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import cvzone
import math
from ultralytics import YOLO


class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'detected_xy', 10)

        # Load YOLO model
        self.model = YOLO('/home/samiul/Thesis_ws/System/Vision_system/mymodel/runs/detect/train4/weights/best.pt')
        self.names = self.model.names

        # Camera capture
        self.cap = cv2.VideoCapture(0)
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"Camera resolution: {int(width)} x {int(height)}")

        # Timer for loop (30 Hz)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def timer_callback(self):
        success, img = self.cap.read()
        if not success:
            self.get_logger().warn("Camera frame not received!")
            return

        results = self.model(img, stream=True)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # Draw bounding box
                w, h = x2 - x1, y2 - y1
                cvzone.cornerRect(img, (x1, y1, w, h), l=9)

                # Calculate normalized coordinates
                sx = 60 / 640
                sy = 32 / 360
                xn = sx * (x1 + x2) / 2
                yn = sy * (y1 + y2) / 2
                X = 30 - xn
                Y = yn - 5
                print(X,Y)

                # Publish X, Y
                msg = Float32MultiArray()
                msg.data = [float(X), float(Y)]
                self.publisher_.publish(msg)

                # Confidence + Class
                conf = math.ceil(box.conf[0] * 100) / 100
                cls_id = int(box.cls[0])
                cvzone.putTextRect(
                    img,
                    f"{self.names[cls_id]} {conf}",
                    (max(0, x1), max(35, y1)),
                    scale=1,
                    thickness=1
                )

        # Show video feed (optional)
        cv2.imshow('YOLO Detection', img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


















        

 
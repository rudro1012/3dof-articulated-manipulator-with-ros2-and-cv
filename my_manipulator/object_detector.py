#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import cvzone
import math
from ultralytics import YOLO


class object_detector(Node):
    def __init__(self):
        super().__init__("object_detector")
        self.publisher=self.create_publisher(Float32MultiArray, 'coordinates',10)     
        
        #defining the detection model
        self.model=YOLO('/home/samiul/Thesis_ws/System/Vision_system/mymodel/runs/detect/train4/weights/best.pt')
        self.names=self.model.names
        self.cap=cv2.VideoCapture(0)
        width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height=self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"camera resolution is {int(width)} x {int(height)}")
        self.timer=self.create_timer(0.1,self.timer_callback)
    
    def timer_callback(self):
        success, img=self.cap.read()
        if not success:
            self.get_logger('camera couldnt be open')
            return
        results=self.model(img,stream=True)
        for r in results:
            boxes=r.boxes
            for box in boxes:
                x1,y1,x2,y2=box.xyxy[0]
                x1,y1,x2,y2= int(x1),int(y1),int(x2),int(y2)
                w=abs(x1-x2)
                h=abs(y1-y2)
                cvzone.cornerRect(img,(x1,y1,w,h),l=8)

                sx=60/640
                sy=32/360
                xn=sx*(x1+x2)/2
                yn=sy*(y1+y2)/2
                X=30-xn
                Y=yn-5
                print(X,Y)


                msg=Float32MultiArray()
                msg.data=[float(X),float(Y)]
                self.publisher.publish(msg)
                
                conf=math.ceil(box.conf[0]*100)/100
                class_id=int(box.cls[0])
                cvzone.putTextRect(img,f"{self.names[class_id]}",(max(0,x1),max,(35,y1)),scale=1,thickness=1)=
        cv2.imshow('Obejct detection',img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node=object_detector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=='__main__':
    main()



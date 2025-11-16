#!/home/samiul/Thesis_ws/tvm/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2
import cvzone
import math
from ultralytics import YOLO
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class object_detector(Node):
    def __init__(self):
        super().__init__("object_detector")
        qos = QoSProfile(
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        history=QoSHistoryPolicy.KEEP_LAST,
                        depth=1
                        )
        self.publisher=self.create_publisher(Float32MultiArray, 'coordinates',1)  

        # checking arm status
        self.busy_status=False
        self.status_subscriber=self.create_subscription(Bool,'status',self.status_callback,1)   
        
        #defining the detection model
        self.model=YOLO('/home/samiul/Thesis_ws/System/Vision_system/mymodel/runs/detect/train4/weights/best.pt')
        self.names=self.model.names

        # fixing camera index
        self.cap=cv2.VideoCapture(0)

        # getting information on fame height and width
        width=self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height=self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"camera resolution is {int(width)} x {int(height)}")

        # getting callback fucntion in every 0.1 second
        self.timer=self.create_timer(0.1,self.timer_callback)


    def status_callback(self, msg):
        self.busy_status=msg.data

    # defining timercallback   
    def timer_callback(self):

        if self.busy_status:  #if arm is busy then the detection will pause
            return
        success, img=self.cap.read()
        if not success:
            self.get_logger('camera couldnt be open')  #checking if camera is working or not
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

                #proportion for equivalent distance on physical surface
                sx=59/640   
                sy=33/360
                xn=sx*(x1+x2)/2
                yn=sy*(y1+y2)/2
                X=28-xn
                Y=yn-5
                self.get_logger().info(f'X:{X}, Y:{Y} ')
                # X=16.0
                # Y=5.0

                # co ordinate message
                position=Float32MultiArray()
                position.data=[float(X),float(Y)]
                self.publisher.publish(position)
                
                # detection confidence for objects
                conf=math.ceil(box.conf[0]*100)/100
                class_id=int(box.cls[0])
                cvzone.putTextRect(img,f"{self.names[class_id]} {conf}",(max(0,x1),max(35,y1)),scale=1,thickness=1)

        cv2.imshow('Obejct detection',img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node=object_detector()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()



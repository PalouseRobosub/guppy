import rclpy
from rclpy.node import Node
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2 as cv
import numpy as np
from guppy_msgs.msg import CornerDetection, CornerDetectionList
from vision_msgs.msg import Point2D
from geometry_msgs.msg import Point

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')

        # Model generously provided by OSU-UWRT
        self.model = YOLO("resource/ffc_rs_26.pt")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, 'cam/d', 10, self.callback, qos_profile_sensor_data)
        self.pub = self.create_publisher(CornerDetectionList, "/cam/d/detections", 10)

    def callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg)
        results = self.model(frame)

        l = CornerDetectionList()

        if results[0].masks is not None:
            json = results[0].to_json()
            for i in results[0].masks.xy:
                contour = i.astype(np.float32)

                rect = cv.minAreaRect(contour)
                box = cv.boxPoints(rect)
                box = np.int32(box)

                for j in box:
                    point = Point2D()
                    point.x = j[0]
                    point.y = j[1]
                    det.corners.append(point)

                det = CornerDetection()
                det.confidence = json[i].confidence
                det.name = json[i].name
                det.square = True

                points = [
                    (-0.08,  0.08, 0),
                    ( 0.08,  0.08, 0),
                    ( 0.08, -0.08, 0),
                    (-0.08, -0.08, 0),
                ]
                
                for x, y, z in points:
                    point = Point()
                    point.x = float(x)
                    point.y = float(y)
                    point.z = float(z)
                    det.dimension_points.append(point)

                l.detections.append(det)

        l.header = frame.header
        self.pub.publish(l)    
            


def main(args=None):
    rclpy.init(args=args)
    objdet = ObjectDetection()
    rclpy.spin(objdet)
    objdet.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

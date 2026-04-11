#!/usr/bin/env python3

import cv2
import rclpy
import apriltag
from rclpy.node import Node
from vision_msgs.msg import Point2D
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from guppy_msgs.msg import CornerDetection, CornerDetectionList

from cv_bridge import CvBridge
bridge = CvBridge()
detector = apriltag.Detector()

class AprilTagDetection(Node):
    def __init__(self):
        super().__init__('apriltag_detector')
        self.subscription = self.create_subscription(Image, '/cam/test/raw', self.callback, 10)
        self.annotation_publisher = self.create_publisher(Image, '/cam/test/annotated', 10)
        self.publisher = self.create_publisher(CornerDetectionList, '/cam/test/detections', 10)

    def callback(self, image):
        img_o = bridge.imgmsg_to_cv2(image)
        img = cv2.cvtColor(img_o, cv2.COLOR_BGR2GRAY)
        results = detector.detect(img)

        msg = CornerDetectionList()

        for result in results:
            detection = CornerDetection()
            detection.confidence = result.decision_margin / 100
            detection.name = f"tag_{result.tag_id}"
            detection.square = True
            
            for x, y in result.corners:
                point = Point2D()
                point.x = x
                point.y = y
                detection.corners.append(point)
            

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
                detection.dimension_points.append(point)

            msg.detections.append(detection)

        self.publisher.publish(msg)

        # annotate...
        image = img_o
        for r in results:
            (ptA, ptB, ptC, ptD) = r.corners
            ptB = (int(ptB[0]), int(ptB[1]))
            ptC = (int(ptC[0]), int(ptC[1]))
            ptD = (int(ptD[0]), int(ptD[1]))
            ptA = (int(ptA[0]), int(ptA[1]))

            cv2.line(image, ptA, ptB, (0, 255, 0), 2)
            cv2.line(image, ptB, ptC, (0, 255, 0), 2)
            cv2.line(image, ptC, ptD, (0, 255, 0), 2)
            cv2.line(image, ptD, ptA, (0, 255, 0), 2)

            (cX, cY) = (int(r.center[0]), int(r.center[1]))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)

            cv2.putText(image, "tag_" + str(r.tag_id), (cX - 5, cY - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 4)
        
        self.annotation_publisher.publish(bridge.cv2_to_imgmsg(image, 'bgr8'))


def main(args=None):
    rclpy.init(args=args)
    apriltagdetector = AprilTagDetection()
    rclpy.spin(apriltagdetector)
    apriltagdetector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
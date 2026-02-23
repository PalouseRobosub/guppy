#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

TARGET_RANGE = 3 * 0.3048

FOV_RAD=1.047

KP_YAW=0.5
KP_RANGE=0.5
KP_VERTICAL=0.3

DEADBAND_PX=5
DEADBAND_RANGE=0.1

MAX_YAW=1.0
MAX_LINEAR=1.0

class FCSTargeting(Node):
    def __init__(self):
        super().__init__('fcs_targeting')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/guppy/FCS_camera',
            self.image_callback,
            10
        )

        self.range_sub = self.create_subscription(
            Float32,
            '/fcs/target_range',
            self.range_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.image_width = 640
        self.image_height = 480
        self.current_range = None

        self.get_logger().info('FCS TARGETING NODE STARTED')
    def range_callback(self, msg: Float32):
        self.current_range = msg.data

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.image_width=frame.shape[1]
        self.image_height=frame.shape[0]
        image_cx=self.image_width /2.0
        image_cy=self.image_height / 2.0
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_red1= np.array([0,120,70])
        upper_red1 = np.array([10,  255, 255])

        lower_red2 = np.array([170, 120,  70])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv,lower_red1,upper_red1)
        mask2  =  cv2.inRange(hsv,lower_red2,upper_red2)
        mask = cv2.bitwise_or(mask1,mask2)

        kernel = np.ones((5,5), np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_DILATE,kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        twist = Twist()

        if contours:
            largest = max(contours,key=cv2.contourArea)
            area = cv2.contourArea(largest)
            if area > 100:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    cx = M['m10'] / M['m00']
                    cy = M['m01'] / M['m00']

                    #CENTERING TARGET HORIZONTALLY
                    error_x = (cx-image_cx) / image_cx
                    # if abs(cx - image_cx) < DEADBAND_PX:
                    #     twist.angular.z = 0.0
                    # else:
                    #     twist.angular.z=float(np.clip(-KP_YAW*error_x, -MAX_YAW, MAX_YAW))

                    if abs(cx - image_cx) < DEADBAND_PX:
                        twist.linear.y = 0.0
                    else:
                        twist.linear.y=float(np.clip(-KP_YAW*error_x, -MAX_LINEAR, MAX_LINEAR))
                    
                    #CENTERING TARGET VERTICALLY   
                    error_y = (cy-image_cy) / image_cy
                    if abs(cy -image_cy) < DEADBAND_PX:
                        twist.linear.z = 0.0
                    else:
                        twist.linear.z = float(np.clip(-KP_VERTICAL*error_y, -MAX_LINEAR, MAX_LINEAR))

                    #POSITIONING X
                    if self.current_range is not None:
                        range_error = self.current_range - TARGET_RANGE
                        if abs(range_error) < DEADBAND_RANGE:
                            twist.linear.x = 0.0
                        else:
                            twist.linear.x = float(np.clip(KP_RANGE * range_error, -MAX_LINEAR, MAX_LINEAR))
                        self.get_logger().info(
                            f'TARGET: ({cx:.0f},{cy:.0f}) | '
                            f'RANGE: {self.current_range:.2f}m | '
                            f'STANDOFF ERROR: {range_error:.2f}m | '
                            f'CMD: x={twist.linear.x:.2f} y={twist.linear.y:.2f} z={twist.linear.z:.2f}'
                            # f'CMD: x={twist.linear.x:.2f} z={twist.linear.z:.2f} yaw={twist.angular.z:.2f}'
                        )
                    else:
                        self.get_logger().warn('WAITING FOR RANGE DATA')

                    if self.current_range is not None:
                        cv2.putText(frame,
                                    f'RANGE: {self.current_range:.2f}m | TARGET: {TARGET_RANGE:.2f}m',
                                    (10,30),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.7,
                                    (0,255,0),
                                    2)
        else:
            self.get_logger().warn("NO TARGETS DETECTED")
        
        self.cmd_vel_pub.publish(twist)
        cv2.imshow('FCS TARGETING',frame)
        cv2.imshow('RED MASK',mask)
        cv2.waitKey(1)
    
def main(args = None):
    rclpy.init(args=args)
    node = FCSTargeting()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
            



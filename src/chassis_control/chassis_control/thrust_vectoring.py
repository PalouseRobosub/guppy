import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist

from math import sin, cos, tan, radians
import numpy as np
from scipy.optimize import lsq_linear, minimize

motors = [
    # corners
    (-1, +1, 0,  0, +135),
    (+1, +1, 0,  0, -135),
    (+1, -1, 0,  0,  -45),
    (-1, -1, 0,  0,  +45),

    # middles
    (+0, +1, 0, 90,    0),
    (+1, +0, 0, 90,    0),
    (+0, -1, 0, 90,    0),
    (-1, +0, 0, 90,    0),
]

def calculate_coefficients(x, y, z, phi, theta):
    p = radians(90-phi)
    t = radians(90+theta)

    sinp = sin(p)
    sint = sin(t)
    cost = cos(t)
    cosp = cos(p)

    return [
        sinp * cost,                    # Fx
        sinp * sint,                    # Fy
        cosp,                           # Fz
        (z*sinp*sint) - (y*cosp),       # Rx
        (x*cosp) - (z*sinp*cost),       # Ry
        (y*sinp*cost) - (x*sinp*sint)   # Rz
    ]

coefficient_matrix = []
for motor in motors:
    coefficient_matrix.append(calculate_coefficients(*motor))


coefficient_matrix = np.array(coefficient_matrix).transpose()

class ThrustVectoring(Node):

    def __init__(self):
        super().__init__('thrust_vectoring')

        self.subscription = self.create_subscription(Twist, '/teleop/chassis_twist', self.callback, 10)
        self.thrust_publisher = self.create_publisher(Float32MultiArray, '/chassis_control/motor_outputs', 10)
        self.twist_publisher = self.create_publisher(Twist, '/chassis_control/output_twist', 10)

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % str(msg))
        desired = np.array([
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z
        ])
        self.get_logger().info('desired: "%s"' % str(desired))

        bounds = ([-1]*len(motors), [1]*len(motors))

        solutions = np.linalg.lstsq(coefficient_matrix, desired, rcond=None)[0]
        def fun(x):
            s = sum(np.abs(np.dot(coefficient_matrix,x)-desired))
            return s
        try:
            sol = minimize(fun, solutions, method='L-BFGS-B', bounds = [(-1,1) for i in range(len(motors))])
        except ValueError as e:
            sol = {"x":[0]*len(motors)}
        solutions = sol['x']

        #solutions = lsq_linear(coefficient_matrix, desired, bounds=bounds).x
        self.get_logger().info('solutions: "%s"' % str(solutions))

        out_thrusts = Float32MultiArray()
        out_thrusts.data = solutions.tolist()
        self.get_logger().info('out: "%s"' % str(out_thrusts))
        self.thrust_publisher.publish(out_thrusts)

        rA = list(map(sum, (solutions * coefficient_matrix).tolist()))
        self.get_logger().info('rA: "%s"' % str(rA))

        out_twist = Twist()
        out_twist.linear.x = rA[0]
        out_twist.linear.y = rA[1]
        out_twist.linear.z = rA[2]
        out_twist.angular.x = rA[3]
        out_twist.angular.y = rA[4]
        out_twist.angular.z = rA[5]

        self.twist_publisher.publish(out_twist)


def main(args=None):
    rclpy.init(args=args)

    node = ThrustVectoring()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

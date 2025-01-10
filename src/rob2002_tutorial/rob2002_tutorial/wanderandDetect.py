import rclpy
from rclpy.node import Node
from rclpy import qos

import cv2 as cv
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Polygon, PolygonStamped, Point32
from cv_bridge import CvBridge



from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from .detector_dblcounting import DetectorBasic
class wander(Node):
    """
    A very simple Roamer implementation for LIMO.
    It goes straight until any obstacle is within
    a specified distance and then just turns left.
    A purely reactive approach.
    """
    def __init__(self):
        """
        On construction of the object, create a Subscriber
        to listen to lasr scans and a Publisher to control
        the robot
        """
        super().__init__('mover_laser')
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)

        self.angular_range = 10
    
    def laserscan_callback(self, data):
        """
        Callback called any time a new laser scan become available
        """
        min_dist = min(data.ranges[int(len(data.ranges)/2) - self.angular_range : int(len(data.ranges)/2) + self.angular_range])
        print(f'Min distance: {min_dist:.2f}')
        msg = Twist()
        if min_dist < 0.7:
            msg.linear.x = 0.0
            msg.angular.z = -0.5
        else:
            msg.linear.x = 0.2
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    mvoer_laser = wander()
    detector_dblcounting = DetectorBasic()
    rclpy.spin(detector_dblcounting)

    mvoer_laser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
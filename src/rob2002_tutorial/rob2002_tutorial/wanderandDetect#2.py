import rclpy
from rclpy.node import Node
from rclpy import qos
import cv2 as cv
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from std_msgs.msg import Header

from  colour_detection import DetectorBasic
class RobotController(Node):
    detection_threshold = 0.2  # in meters    

    def init(self):
        super().__init__('robot_controller' , 'detector_basic')

        # Image and object detection setup
        self.bridge = CvBridge()
        self.detected_objects = []  # List of detected objects

    # Robot movement setup
        self.publisher_cmd_vel = self.create_publisher(Twist, '/cmd_vel', qos.qos_profile_system_default)

    # Subscriptions
        self.subscriber_image = self.create_subscription(Image, '/camera/image_raw', self.image_callback, qos.qos_profile_sensor_data)
        self.subscriber_laser = self.create_subscription(LaserScan, '/scan', self.laserscan_callback, qos.qos_profile_sensor_data)
        self.subscriber_object = self.create_subscription(PoseStamped, '/object_location', self.counter_callback, qos.qos_profile_sensor_data)

    # Publisher for object count
        self.publisher_objects = self.create_publisher(PoseArray, '/object_count_array', qos.qos_profile_system_default)

        # Wandering state
        self.obstacle_detected = False
        self.angular_range = 10

    def image_callback(self, data):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')

        # Convert BGR to HSV
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # Define range for red color in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Mask for red color
        mask1 = cv.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        # Find contours of red objects
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # Calculate the bounding box
            x, y, w, h = cv.boundingRect(contour)
            # Filter small objects
            if w * h > 500:  # Adjust size threshold as needed
                # Calculate the center of the red object
                cx = x + w // 2
                cy = y + h // 2

                # Assume z=0 for simplicity or add depth estimation here
                self.get_logger().info(f"Red object detected at image position: x={cx}, y={cy}")
                pose = PoseStamped()
                pose.pose.position.x = cx  # Replace with actual x-coordinate if available
                pose.pose.position.y = cy  # Replace with actual y-coordinate if available
                pose.pose.position.z = 0.0  # Add depth if available
                self.counter_callback(pose)

    def counter_callback(self, data):
        new_object = data.pose

        # Check if the new object is far enough from all detected objects
        object_exists = False
        for object in self.detected_objects:
            pos_a = object.position
            pos_b = new_object.position
            d = math.sqrt((pos_a.x - pos_b.x) ** 2 + (pos_a.y - pos_b.y) ** 2 + (pos_a.z - pos_b.z) ** 2)
            if d < self.detection_threshold:  # Found a close neighbor
                object_exists = True
                break

        if not object_exists:  # New object detected
            self.detected_objects.append(new_object)
            self.get_logger().info(f"New red object detected at: x={new_object.position.x:.2f}, y={new_object.position.y:.2f}, z={new_object.position.z:.2f}")

        # Publish all detected objects as a PoseArray
        parray = PoseArray(header=Header(frame_id=data.header.frame_id))
        for object in self.detected_objects:
            parray.poses.append(object)
        self.publisher_objects.publish(parray)

        # Log detected objects to terminal
        self.get_logger().info(f'Total red objects detected: {len(self.detected_objects)}')
        for idx, obj in enumerate(self.detected_objects, 1):
            self.get_logger().info(f'Object {idx}: x={obj.position.x:.2f}, y={obj.position.y:.2f}, z={obj.position.z:.2f}')

    def laserscan_callback(self, data):
        total_ranges = len(data.ranges)
        left_range = data.ranges[:total_ranges // 3]
        front_range = data.ranges[total_ranges // 3: 2 * total_ranges // 3]
        right_range = data.ranges[2 * total_ranges // 3:]

        min_dist_left = min(left_range)
        min_dist_front = min(front_range)
        min_dist_right = min(right_range)

        min_dist = min(data.ranges[int(len(data.ranges) / 2) - self.angular_range:int(len(data.ranges) / 2) + self.angular_range])
        self.get_logger().info(f'Front: {min_dist_front:.2f}, Left: {min_dist_left:.2f}, Right: {min_dist_right:.2f}')

        msg = Twist()
        if min_dist < 1.0:
            if min_dist_left >= min_dist_right:
                msg.linear.x = 0.0
                msg.angular.z = -0.5
                self.get_logger().info("Turning left")
            elif min_dist_left < min_dist_right:
                msg.linear.x = 0.0
                msg.angular.z = 0.5
                self.get_logger().info("Turning right")
        else:
            self.get_logger().info("Moving forward")
            msg.linear.x = 0.2
        self.publisher_cmd_vel.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    colour_detector = colour_detector()
    rclpy.spin(colour_detector)
    rclpy.spin(robot_controller)
    colour_detector.destroy_node()
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



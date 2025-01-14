import rclpy
from rclpy.node import Node
from rclpy import qos

import cv2 as cv
import time 
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from cv_bridge import CvBridge

from rectangle import Rectangle

class DetectorBasic(Node):
    visualisation = True
    data_logging = False
    log_path = 'evaluation/data/'
    seq = 0
    prev_objects = []
   
import rclpy
from rclpy.node import Node
from rclpy import qos

import cv2 as cv
import time 
import math
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, PolygonStamped, Point32
from cv_bridge import CvBridge

from rectangle import Rectangle

class DetectorBasic(Node):
  visualisation = True
  data_logging = False
  log_path = 'evaluation/data/'
  seq = 0
  prev_objects = []
  

  def __init__(self):    
        super().__init__('detector_basic')
        self.bridge = CvBridge()
        self.run = False
        self.min_area_size = 100.0
        self.countour_color = (255, 255, 0) # cyan
        self.countour_width = 1 # in pixels
        self.object_counter = 0
        self.run = True
        self.object_pub = self.create_publisher(PolygonStamped, '/object_polygon', 10)
        self.image_sub = self.create_subscription(Image, '/limo/depth_camera_link/image_raw', 
                                                  self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)

  def image_color_callback(self, data):
    bgr_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  # convert ROS Image message to OpenCV format

    # Detect a color blob in the color image
    bgr_thresh = cv.inRange(bgr_image, (0, 0, 80), (50, 50, 255))

    # Find all separate image regions in the binary image
    bgr_contours, _ = cv.findContours(
        bgr_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE
    )

    detected_objects = []
    for contour in bgr_contours:
        area = cv.contourArea(contour)
        if area > self.min_area_size:  # Detect only large objects
            bbx, bby, bbw, bbh = cv.boundingRect(contour)
            detected_objects.append(Rectangle(bbx, bby, bbx + bbw, bby + bbh))
            if self.visualisation:
                cv.rectangle(
                    bgr_image, (bbx, bby), (bbx + bbw, bby + bbh), self.countour_color, self.countour_width
                )

    # Keep a count of unique objects
    unique_objects = []
    for rectA in detected_objects:
        is_unique = True
        for rectB in unique_objects:
            if rectA & rectB:  # Overlap detection using the & operator
                is_unique = False
                break
        if is_unique:
            unique_objects.append(rectA)

    # Convert unique detections to Polygon messages
    polygons = [
        Polygon(points=[
            Point32(x=float(obj.x1), y=float(obj.y1)),
            Point32(x=float(obj.x2), y=float(obj.y1)),
            Point32(x=float(obj.x2), y=float(obj.y2)),
            Point32(x=float(obj.x1), y=float(obj.y2))
        ]) for obj in unique_objects
    ]

    # Publish detected objects as PolygonStamped messages
    for polygon in polygons:
        self.object_pub.publish(PolygonStamped(polygon=polygon, header=data.header))

    # Log the number of unique detections
    num_unique_objects = len(unique_objects)
    print(f'Detected {num_unique_objects} unique object(s) in the frame.')

    # Update previous objects for consistency in future frames
    self.prev_objects = unique_objects

    # Logging and visualization
    if self.data_logging:
        cv.imwrite(self.log_path + f'colour_{self.seq:06d}.png', bgr_image)
        cv.imwrite(self.log_path + f'mask_{self.seq:06d}.png', bgr_thresh)

    if self.visualisation:
        cv.imshow("colour image", bgr_image)
        cv.imshow("detection mask", bgr_thresh)
        cv.waitKey(1)

    self.seq += 1
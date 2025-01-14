# Python libs
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy import qos
import math

# OpenCV
import cv2

# ROS libraries
import image_geometry
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from cv_bridge import CvBridge

# ROS Messages
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo 
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion , PoseArray
import cv2 as cv

# System
import sys

from scipy.spatial import distance  # Add this at the top of your code

class Detector3D(Node):
    # use the real robot?
    real_robot = False

    ccamera_model = None
    dcamera_model = None
    image_depth_ros = None
    # aspect ratio between the color and depth cameras
    # calculated as (color_horizontal_FOV/color_width) / (depth_horizontal_FOV/depth_width)
    # in camera_info callbacks
    color2depth_aspect = None

    min_area_size = 100
    global_frame = 'odom' # change to 'map' if using maps

    visualisation = True

    def __init__(self):    
        super().__init__('Detector3D')
        self.bridge = CvBridge()
        self.total_objects = 5
        self.detected_objects = []  # List to track detected objects' global positions
        self.min_distance = 0.5  # Minimum distance to consider objects as separate
        self.run = False

        # subscribers and publishers
        ccamera_info_topic = '/limo/depth_camera_link/camera_info'
        dcamera_info_topic = '/limo/depth_camera_link/depth/camera_info'
        cimage_topic = '/limo/depth_camera_link/image_raw'
        dimage_topic = '/limo/depth_camera_link/depth/image_raw'
        self.camera_frame = 'depth_link' 

        if self.real_robot:
            ccamera_info_topic = '/camera/color/camera_info'
            dcamera_info_topic = '/camera/depth/camera_info'
            cimage_topic = '/camera/color/image_raw'
            dimage_topic = '/camera/depth/image_raw'
            self.camera_frame = 'camera_color_optical_frame'

        self.ccamera_info_sub = self.create_subscription(CameraInfo, ccamera_info_topic,
                                                self.ccamera_info_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.dcamera_info_sub = self.create_subscription(CameraInfo, dcamera_info_topic,
                                                self.dcamera_info_callback, qos_profile=qos.qos_profile_sensor_data)

        self.cimage_sub = self.create_subscription(Image, cimage_topic, 
                                                  self.image_color_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.dimage_sub = self.create_subscription(Image, dimage_topic, 
                                                  self.image_depth_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.object_location_pub = self.create_publisher(PoseStamped, '/object_location', qos.qos_profile_parameters)

                # subscribe to object detector
       # self.subscriber = self.create_subscription(PoseStamped, '/object_location',  self.counter_callback,qos_profile=qos.qos_profile_sensor_data)
        
        # publish all detected object as an array of poses
        #self.publisher = self.create_publisher(PoseArray, '/object_count_array', qos.qos_profile_parameters)

        # tf functionality
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def color2depth_calc(self):
        if self.color2depth_aspect is None and self.ccamera_model and self.dcamera_model:
            self.color2depth_aspect = (math.atan2(self.ccamera_model.width, 2 * self.ccamera_model.fx()) / self.ccamera_model.width) \
                / (math.atan2(self.dcamera_model.width, 2 * self.dcamera_model.fx()) / self.dcamera_model.width)

    def image2camera_tf(self, image_coords, image_color, image_depth):
        # transform" from color to depth coordinates
        depth_coords = np.array(image_depth.shape[:2])/2 + (np.array(image_coords) - np.array(image_color.shape[:2])/2)*self.color2depth_aspect
        # get the depth reading at the centroid location
        depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!
        # calculate object's 3d location in camera coords
        camera_coords = np.array(self.ccamera_model.projectPixelTo3dRay((image_coords[1], image_coords[0]))) #project the image coords (x,y) into 3D ray in camera coords 
        camera_coords /= camera_coords[2] # adjust the resulting vector so that z = 1
        camera_coords = camera_coords*depth_value # multiply the vector by depth
        pose = Pose(position=Point(x=camera_coords[0], y=camera_coords[1], z=camera_coords[2]), 
                    orientation=Quaternion(w=1.0))
        return pose
    
    def ccamera_info_callback(self, data):
        if self.ccamera_model is None:
            self.ccamera_model = image_geometry.PinholeCameraModel()
            self.ccamera_model.fromCameraInfo(data)
            self.color2depth_calc()

    def dcamera_info_callback(self, data):
        if self.dcamera_model is None:
            self.dcamera_model = image_geometry.PinholeCameraModel()
            self.dcamera_model.fromCameraInfo(data)
            self.color2depth_calc()

    def image_depth_callback(self, data):
        self.image_depth_ros = data



    # def counter_callback(self, data):
    #     new_object = data.pose
    #     parray = PoseArray(header=Header(frame_id=data.header.frame_id))
    #     for object in self.detected_objects:
    #         parray.poses.append(object)
    #     self.publisher.publish(parray) 



    def image_color_callback(self, data):
        if self.color2depth_aspect is None or self.image_depth_ros is None:
            return
        
        # Convert image to OpenCV format
        self.image_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, "32FC1")
        if self.real_robot:
            self.image_depth /= 1000.0

        red_thresh = cv.inRange(self.image_color, (0, 0, 80), (50, 50, 255))
        green_thresh = cv.inRange(self.image_color, (0, 80, 0), (50, 255, 50))
        blue_thresh = cv.inRange(self.image_color, (80, 0, 0), (255, 50, 50))
        combined_thresh = cv.bitwise_or(cv.bitwise_or(red_thresh, green_thresh), blue_thresh)
        object_contours, _ = cv.findContours(combined_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        for num, cnt in enumerate(object_contours):  
            area = cv2.contourArea(cnt)
            if area > self.min_area_size:
                cmoms = cv2.moments(cnt)
                image_coords = (cmoms["m01"] / cmoms["m00"], cmoms["m10"] / cmoms["m00"])
                camera_pose = self.image2camera_tf(image_coords, self.image_color, self.image_depth)
                global_pose = do_transform_pose(camera_pose, 
                                                self.tf_buffer.lookup_transform(self.global_frame, self.camera_frame, rclpy.time.Time()))
                global_position = (global_pose.position.x, global_pose.position.y, global_pose.position.z)        
                is_new_object = True
                for prev_pos in self.detected_objects:
                 if distance.euclidean(global_position, prev_pos) < self.min_distance:
                    is_new_object = False
                    break

        #    If the object is new, add it to the list
                if is_new_object:
                   self.detected_objects.append(global_position)
                print(f"--- object id {num} ---")
                print(f'Total objects detected: {len(self.detected_objects)-1}')
                print(f'Image coords: {image_coords}')
                print(f'Camera coords: {camera_pose.position}')
                print(f'Global coords: {global_pose.position}')

                    # Print message if all objects are detected
                if len(self.detected_objects) == self.total_objects:
                        self.run = True
                        print("All objects are detected!")

                if self.visualisation:
                        cv2.circle(self.image_color, (int(image_coords[1]), int(image_coords[0])), 5, 255, -1)

        if self.visualisation:
            self.image_depth *= 1.0 / 10.0
            self.image_color = cv2.resize(self.image_color, (0, 0), fx=1, fy=1)
            self.image_depth = cv2.resize(self.image_depth, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow("image color", self.image_color)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    image_projection = Detector3D()
    rclpy.spin(image_projection)
    image_projection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

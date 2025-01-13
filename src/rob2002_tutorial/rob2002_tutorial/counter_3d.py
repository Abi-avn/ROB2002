import rclpy
from rclpy.node import Node
from rclpy import qos
import math
import sys
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseArray

class Counter3D(Node):
    detection_threshold = 1.0 # in meters 

    def __init__(self):      
        super().__init__('counter_3d')
        self.total_objects = 5
        self.detected_objects = [] # list of all detected objects
        self.run = False
        # subscribe to object detector
        self.subscriber = self.create_subscription(PoseStamped, '/object_location', 
                                                   self.counter_callback,
                                                   qos_profile=qos.qos_profile_sensor_data)
        
        # publish all detected object as an array of poses
        self.publisher = self.create_publisher(PoseArray, '/object_count_array',
                                                      qos.qos_profile_parameters)

    def counter_callback(self, data):
        new_object = data.pose
        # check if the new object is away from all detected objects so far

        object_exists = False
        for object in self.detected_objects:
            # calculate the distance between the new_object and each in the list
            pos_a = object.position
            pos_b = new_object.position
            d = math.sqrt((pos_a.x - pos_b.x) ** 2 + (pos_a.y - pos_b.y) ** 2 + (pos_a.z - pos_b.z) ** 2)
            if d < self.detection_threshold: # found a close neighbour in the already existing list, so this one won't be added
                object_exists = True
                break

        if not object_exists: # new object!
            self.detected_objects.append(new_object)

        # publish a PoseArray of object poses for visualisation in rviz
        parray = PoseArray(header=Header(frame_id=data.header.frame_id))
        for object in self.detected_objects:
            parray.poses.append(object)
        self.publisher.publish(parray)            

        # print to the console
        print(f'total count {len(self.detected_objects)}')
        if self.total_objects <= len(self.detected_objects):
            print("all objects detected")
            self.run = True
            sys.exit(0)
        for object in self.detected_objects:
            print(object.position)

def main(args=None):
    rclpy.init(args=args)
    counter_3d = Counter3D()
    if counter_3d.run == True : 
     counter_3d.destroy_node()
     rclpy.shutdown()
     sys.exit(0)

    else:
     rclpy.spin(counter_3d)



if __name__ == '__main__':
    main()
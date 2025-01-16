import rclpy
from rclpy.node import Node
import time 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from detector import DetectorBasic

class wander(Node):
    def __init__(self):

        super().__init__('mover_laser')
        #movement publisher
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        #LIDAR data
        self.subscriber = self.create_subscription(LaserScan, "/scan", self.laserscan_callback, 10)

        self.angular_range = 10
    
    def laserscan_callback(self, data):
        total_ranges = len(data.ranges) ; 
        #Split the LIDAR rays into 2 parts 
        left_range = data.ranges[:total_ranges //3] #left side
        right_range = data.ranges[2*total_ranges //3:] #right side
        #Workout the minimum distance from the split ranges
        min_dist_left = min(left_range)
        min_dist_right = min (right_range)
        #Turning threshold 
        min_dist = min(data.ranges[int(len(data.ranges)/2) - self.angular_range : int(len(data.ranges)/2) + self.angular_range])
    
        msg = Twist()
        #movement logic
        if min_dist< 1.0:
            if(min_dist_left >= min_dist_right):
             msg.linear.x = 0.0
             msg.angular.z = -0.5
             time.sleep(0.1)# delay for turning
            
            elif (min_dist_left <= min_dist_right):
                msg.linear.x = 0.0
                msg.angular.z = 0.5
                time.sleep(0.1) # delay for turning
            else:
                 msg.linear.x = 0.0
        else:
            msg.linear.x = 0.2
        self.publisher.publish(msg )


def main(args=None):
    rclpy.init(args=args)

    # Create instances of the nodes
    mover = wander()
    detector_3d = DetectorBasic()


    # Use a MultiThreadedExecutor to spin the nodes
    executor = MultiThreadedExecutor()
    executor.add_node(detector_3d)
    executor.add_node(mover)

    try:
        # Continuously spin the executor until detector_3d.run becomes True
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)  # Allow other tasks to run
            if detector_3d.run:  # Check if the condition to exit is met
                 print("All objects detected. Shutting down...")
                 break
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        # Clean up resources
        detector_3d.destroy_node()
        mover.destroy_node()
        rclpy.shutdown()
        print("Program terminated.")
if __name__ == '__main__':
    main()

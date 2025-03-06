
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
import math
import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from .robot_navigator import BasicNavigator # Helper module


class Navi(Node):
    def __init__(self):
        super().__init__('Navi')

        
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0

    def timer_callback(self):
        msg = PoseWithCovarianceStamped()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    
    def publish(self, x, y, w):


        initpose = PoseWithCovarianceStamped()
        initpose.header.stamp = self.get_clock().now().to_msg()
        initpose.header.frame_id = "map"
        initpose.pose.pose.position.x = x
        initpose.pose.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, w)

        initpose.pose.pose.orientation.x = quaternion[0]
        initpose.pose.pose.orientation.y = quaternion[1]
        initpose.pose.pose.orientation.z = quaternion[2]
        initpose.pose.pose.orientation.w = quaternion[3]
        self.publisher_.publish(initpose)


        self.get_logger().info('Publishing: 123')
    

    def configure_init_pose(self, x, y, w):
        initpose = PoseWithCovarianceStamped()
        initpose.header.stamp = self.get_clock().now().to_msg()
        initpose.header.frame_id = "map"
        initpose.pose.pose.position.x = x
        initpose.pose.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, w)

        initpose.pose.pose.orientation.x = quaternion[0]
        initpose.pose.pose.orientation.y = quaternion[1]
        initpose.pose.pose.orientation.z = quaternion[2]
        initpose.pose.pose.orientation.w = quaternion[3]
        self.publisher_.publish(initpose)


        self.get_logger().info('initial pose configured')
    


    @staticmethod
    def set_goal_pose(x, y, w, time):

        quaternion = quaternion_from_euler(0, 0, w)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = time
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = quaternion[0]
        goal_pose.pose.orientation.y = quaternion[1]
        goal_pose.pose.orientation.z = quaternion[2]
        goal_pose.pose.orientation.w = quaternion[3]
  

        return goal_pose



    # @staticmethod
    # def quaternion_from_euler(roll, pitch, yaw):
    #     """
    #     Converts euler roll, pitch, yaw to quaternion (w in last place)
    #     quat = [x, y, z, w]
    #     Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    #     """
    #     cy = math.cos(yaw * 0.5)
    #     sy = math.sin(yaw * 0.5)
    #     cp = math.cos(pitch * 0.5)
    #     sp = math.sin(pitch * 0.5)
    #     cr = math.cos(roll * 0.5)
    #     sr = math.sin(roll * 0.5)

    #     q = [0] * 4
    #     q[0] = cy * cp * cr + sy * sp * sr
    #     q[1] = cy * cp * sr - sy * sp * cr
    #     q[2] = sy * cp * sr + cy * sp * cr
    #     q[3] = sy * cp * cr - cy * sp * sr

    #     return q


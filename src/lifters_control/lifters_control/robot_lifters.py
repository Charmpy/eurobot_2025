import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from ros_gz_interfaces.srv import SetEntityPose
import subprocess

import gz.transport13 as gz_transport  # Gazebo Transport API
from ros_gz_interfaces.srv import SetEntityPose
# from gz.trajectory_msgs10.pose_pb2 import Pose as GzPose  # Сообщение Pose
# from gz.msgs10.pose_pb2 

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.header_pb2 import Header  # Заголовок сообщения
from gz.msgs.double_pb2 import Double
from gz.msgs.model_pb2 import Model
from gz.msgs10.joint_trajectory_pb2 import JointTrajectory

from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster


import math

class BoardLiftNode(Node):

    def __init__(self):
        super().__init__('board_lift_node')

        self.client = gz_transport.Node()  # Создаём ноду Gazebo Transport
        self.set_model_pos = self.client.advertise('/board_lift_topic', Double)  # Публикуем скорость
        
    def publish(self, data):
        msg = Double()  
        msg.data = data  
        self.set_model_pos.publish(msg)
        self.get_logger().info('Board lift set: "%s"' % msg.data)

class CanLiftNode(Node):

    def __init__(self):
        super().__init__('can_lift_node')

        self.client = gz_transport.Node()  # Создаём ноду Gazebo Transport
        self.set_model_pos = self.client.advertise('/can_lift_topic', Double)  # Публикуем скорость
        
    def publish(self, data):
        msg = Double()  
        msg.data = data  
        self.set_model_pos.publish(msg)
        self.get_logger().info('Can lift set: "%s"' % msg.data)

class CanRotator1Node(Node):

    def __init__(self):
        super().__init__('can_rotator_1_node')

        self.client = gz_transport.Node()  # Создаём ноду Gazebo Transport
        self.set_model_pos = self.client.advertise('/can_rotator_1_topic', Double)  # Публикуем скорость
        
    def publish(self, data):
        msg = Double()  
        msg.data = data  
        self.set_model_pos.publish(msg)
        self.get_logger().info('Can 1 rotation set: "%s"' % msg.data)

class CanRotator2Node(Node):

    def __init__(self):
        super().__init__('can_rotator_2_node')

        self.client = gz_transport.Node()  # Создаём ноду Gazebo Transport
        self.set_model_pos = self.client.advertise('/can_rotator_2_topic', Double)  # Публикуем скорость
        
    def publish(self, data):
        msg = Double()  
        msg.data = data  
        self.set_model_pos.publish(msg)
        self.get_logger().info('Can 2 rotation set: "%s"' % msg.data)
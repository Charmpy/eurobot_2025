# В этом файле содержатся вспомогательные файлы для управления роботом

from std_msgs.msg import String, Empty

import rclpy
from rclpy.node import Node
import time
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

# Это класс для управления захватами из главной ветки программы
# Таких будет два, один для симуляции, другой для реала

class Gripper(Node):
    # gripper*_prefix - префикс топика захватов(часть до attach/detach)
    # s - side, c - center
    def __init__(self, grippers_prefix = ['detachable_jointsl', 'detachable_jointcl', 'detachable_jointcr', 'detachable_jointsr', 'detachable_jointbd'],
                  model = ''):
        # self.node = node  
        super().__init__('twist_pub')

        self.grippers_full_prefix=[]
        self.publishers_attach = []
        self.publishers_detach = []
        self.subscriptions_status = []

        for i in grippers_prefix:
            self.grippers_full_prefix.append(f'{model}/{i}')

            self.publishers_attach.append(self.create_publisher(String, self.grippers_full_prefix[-1]+'/attach', 10))
            self.publishers_detach.append(self.create_publisher(Empty, self.grippers_full_prefix[-1]+'/detach', 10))
            self.subscriptions_status.append(self.create_subscription(String, self.grippers_full_prefix[-1]+'/output',self.on_status_msg,10))

        self.status = []

        self.msg = String()

    
    def grip_sides(self, namel='', namer=''):
        if namel != '':
            self.msg.data = namel
            self.publishers_attach[0].publish(self.msg)
        if namer != '':
            self.msg.data = namer
            self.publishers_attach[3].publish(self.msg)
    
    def grip_center(self, namel='', namer=''):
        if namel != '':
            self.msg.data = namel
            self.publishers_attach[1].publish(self.msg)
        if namer != '':
            self.msg.data = namer
            self.publishers_attach[2].publish(self.msg)
    
    def release_sides(self):
        self.publishers_detach[0].publish(Empty())
        self.publishers_detach[3].publish(Empty())
    
    def release_center(self):
        self.publishers_detach[1].publish(Empty())
        self.publishers_detach[2].publish(Empty())

    def grip_board(self, name=''):
        if name != '':
            self.msg.data = name
            self.publishers_attach[4].publish(self.msg)
    
    def release_board(self):
        self.publishers_detach[4].publish(Empty())

    def on_status_msg(self, data):
        self.status = data.data

    def get(self):
        return self.status

    def __str__(self):
        s = f"Gripper {self.name} position: {self.status}\n"
        return s

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
    
# Этот класс служит для написания типа "макросов" для движений группами захватов 
# и актуаторов одна функция клааса должна выполнять полноценное действите по захвату
# по сути, это просто более высокий уровень организации, сделан для практичности   

class RobotMacros(Node):
    def __init__(self):
        super().__init__('robot_macros')
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.command_pub = self.create_publisher(String, 'grippers', 10)
        self.position_pub = self.create_publisher(String, 'positioning', 10)
    

    def com_compile(self):
        self.get_logger().info('compile start')
        msg = String()
        msg.data = "compile"
        self.command_pub.publish(msg)
        time.sleep(4)
        self.get_logger().info('compile done')

    def com_position(self):
        self.get_logger().info('positioning start')
        msg = String()
        msg.data = "both"
        self.position_pub.publish(msg)
        time.sleep(5)
        self.get_logger().info('positioning done')

    def time_move(self, speed, t):
        msg = Twist()
        msg.linear.x = speed
        self.twist_pub.publish(msg)
        time.sleep(t)
        msg = Twist()
        self.twist_pub.publish(msg)
        time.sleep(0.5)

    def com_build(self):
        msg = String()
        self.get_logger().info('build start')
        msg.data = "build"
        self.command_pub.publish(msg)
        time.sleep(5.5)
        self.get_logger().info('build done')

    def com_start_state(self):
        msg = String()
        msg.data = "start_state"
        self.get_logger().info('start_state start')
        self.command_pub.publish(msg)
        time.sleep(5)
        self.get_logger().info('start_state done')

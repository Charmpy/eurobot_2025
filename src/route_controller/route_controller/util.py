# В этом файле содержатся вспомогательные файлы для управления роботом

from std_msgs.msg import String, Empty

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

# Это класс для управления захватами из главной ветки программы
# Таких будет два, один для симуляции, другой для реала

class Gripper(Node):
    # gripper*_prefix - префикс топика захватов(часть до attach/detach)
    # s - side, c - center
<<<<<<< HEAD
<<<<<<< HEAD
    def __init__(self, node, grippers_prefix = ['detachable_jointsl', 'detachable_jointcl', 
                                                'detachable_jointcr', 'detachable_jointsr',
                                                'detachable_jointbd'],
                  model = ''):
        self.node = node
=======
    def __init__(self, node, grippers_prefix = ['detachable_jointsl', 'detachable_jointcl', 'detachable_jointcr', 'detachable_jointsr'],
=======
    def __init__(self, node, grippers_prefix = ['detachable_jointsl', 'detachable_jointcl', 'detachable_jointcr', 'detachable_jointsr', 'detachable_jointbd'],
>>>>>>> 5b01510 (add board lift)
                  model = ''):
        self.node = node

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

<<<<<<< HEAD
=======
        # self.publisher_sl_attach = self.node.create_publisher(String, self.grippers_full_prefix[0]+'/attach', 10)
        # self.publisher_cl_attach = self.node.create_publisher(String, self.grippers_full_prefix[1]+'/attach', 10)
        # self.publisher_cr_attach = self.node.create_publisher(String, self.grippers_full_prefix[2]+'/attach', 10)
        # self.publisher_sr_attach = self.node.create_publisher(String, self.grippers_full_prefix[3]+'/attach', 10)

        # self.publisher_sl_detach = self.node.create_publisher(String, self.grippers_full_prefix[0]+'/detach', 10)
        # self.publisher_cl_detach = self.node.create_publisher(String, self.grippers_full_prefix[1]+'/detach', 10)
        # self.publisher_cr_detach = self.node.create_publisher(String, self.grippers_full_prefix[2]+'/detach', 10)
        # self.publisher_sr_detach = self.node.create_publisher(String, self.grippers_full_prefix[3]+'/detach', 10)

>>>>>>> 2608916 (grip_test and util.Gripper)
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

class RobotMacros:
    def __init__(self):

        self.gripper = Gripper()
        self.board_lift_node = BoardLiftNode()
        self.can_lift_node = CanLiftNode()
        self.can_rotator_1_node = CanRotator1Node()
        self.can_rotator_2_node = CanRotator2Node()    

    def grip_cans(self, point=1):
        print(1)
        self.gripper.grip_center(namel=f'cylinder{point}2', namer=f'cylinder{point}3')
        self.gripper.grip_sides(namel=f'cylinder{point}1', namer=f'cylinder{point}4')
        self.gripper.grip_board(name=f'box{point}1')

    def grip_release(self):
        self.gripper.release_center()
        self.gripper.release_sides()
        self.gripper.release_board()
    





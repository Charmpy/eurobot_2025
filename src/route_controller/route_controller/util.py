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
    BOARD_LIFT_MIN = 0
    BOARD_LIFT_MAX = 0.17
    CAN_LIFT_MIN = 0
    CAN_LIFT_MAX = 0.19
    CAN_ROTATION_MIN = 0
    CAN_ROTATION_MAX = 0.04
    def __init__(self):
        super().__init__('robot_macros')
        self.gripper = Gripper()
        self.board_lift_node = BoardLiftNode()
        self.can_lift_node = CanLiftNode()
        self.can_rotator_1_node = CanRotator1Node()
        self.can_rotator_2_node = CanRotator2Node()
        self.board_lift_pos = 0
        self.can_lift_pos = 0
        self.can_rotator_1_pos = 0
        self.can_rotator_2_pos = 0
        self.twist_pub = self.create_publisher(Twist, 'cmd_vel', 10)
    

    def grip_cans(self, point=1):
        self.get_logger().info('grip all cans')
        self.gripper.grip_center(namel=f'cylinder{point}2', namer=f'cylinder{point}3')
        self.gripper.grip_sides(namel=f'cylinder{point}1', namer=f'cylinder{point}4')
        self.gripper.grip_board(name=f'box{point}2')
        time.sleep(0.5)

    def grip_release(self):
        self.get_logger().info('release all cans')
        self.gripper.release_center()
        self.gripper.release_sides()
        self.gripper.release_board()
    
    def time_move(self, speed, t):
        msg = Twist()
        msg.linear.x = speed
        self.twist_pub.publish(msg)
        time.sleep(t)
        msg = Twist()
        self.twist_pub.publish(msg)
        time.sleep(0.5)

    def move_up(self):
        msg = Twist()
        self.right_can_rotator_move("in")
        time.sleep(0.1)
        self.left_can_rotator_move("in")
        time.sleep(0.1)
        self.gripper.release_sides()
        time.sleep(0.1)
        self.time_move(-0.3, 0.3)
        self.board_lift_move("up")
        time.sleep(0.1)
        self.can_lift_move('up')
        time.sleep(0.3)
        self.right_can_rotator_move("out")
        time.sleep(0.1)
        self.left_can_rotator_move("out")
        time.sleep(0.1)
        self.time_move(0.3, 0.3)
        time.sleep(0.5)
        self.grip_release()
        time.sleep(0.3)
        self.start_pose()
        time.sleep(0.3)



    def start_pose(self):
        self.right_can_rotator_move("out")
        self.left_can_rotator_move("out")
        time.sleep(0.5)
        self.can_lift_move('down')
        time.sleep(0.5)
    
    def make_second(self):
        # self.can_lift_node.
        self.get_logger().info('2 floor done')
    
    def board_lift_move(self, direction):
        if(direction == "up"):
            self.board_lift_pos = self.BOARD_LIFT_MAX
            self.board_lift_node.publish(self.board_lift_pos)
        elif(direction == "down"):
            self.board_lift_pos = self.BOARD_LIFT_MIN
            self.board_lift_node.publish(self.board_lift_pos)
    
    def can_lift_move(self, direction):
        if(direction == "up"):
            self.can_lift_pos = self.CAN_LIFT_MAX
            self.can_lift_node.publish(self.can_lift_pos)

        elif(direction == "down"):
            self.can_lift_pos = self.CAN_LIFT_MIN
            self.can_lift_node.publish(self.can_lift_pos)

    def right_can_rotator_move(self, direction):
        if(direction == "in"):
            self.can_rotator_1_pos = self.CAN_ROTATION_MIN
            self.can_rotator_1_node.publish(self.can_rotator_1_pos)
        elif(direction == "out"):
            self.can_rotator_1_pos = self.CAN_ROTATION_MAX
            self.can_rotator_1_node.publish(self.can_rotator_1_pos)

    def left_can_rotator_move(self, direction):
        if(direction == "in"):
            self.can_rotator_2_pos = self.CAN_ROTATION_MIN
            self.can_rotator_2_node.publish(self.can_rotator_2_pos)
        elif(direction == "out"):
            self.can_rotator_2_pos = self.CAN_ROTATION_MAX
            self.can_rotator_2_node.publish(self.can_rotator_2_pos)

    def get_joint_pos(self, index):
        if(index == "board"):
            return (self.board_lift_pos)
        if(index == "can_lift"):
            return (self.can_lift_pos)
        if(index == "can_1_angle"):
            return (self.can_rotator_1_pos)
        if(index == "can_2_angle"):
            return (self.can_rotator_2_pos)
        

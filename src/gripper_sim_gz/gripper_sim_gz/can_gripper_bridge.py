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

class GripperBridgeNode(Node):

    def __init__(self):
        super().__init__('gripper_bridge_node')

        self.client = gz_transport.Node()  # Создаём ноду Gazebo Transport
        self.set_model_speed_1 = self.client.advertise('/can_gripper_joint_topic_1', Double)  # Публикуем скорость
        self.set_model_speed_2 = self.client.advertise('/can_gripper_joint_topic_2', Double)  # Публикуем скорость
        
        self.subscription = self.create_subscription(
            Bool,
            'gripper_command',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        joint_msg = Double()    

        if msg.data == True:
            joint_msg.data = 1.57
        else:
            joint_msg.data = 0.0

        self.set_model_speed_1.publish(joint_msg)
        self.set_model_speed_2.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GripperBridgeNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

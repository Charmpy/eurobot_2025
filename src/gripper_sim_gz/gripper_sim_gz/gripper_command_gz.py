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

class GripperCommandNode(Node):

    def __init__(self):
        super().__init__('gripper_command_node')

        self.publisher_ = self.create_publisher(Bool, 'gripper_command', 10)
        
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.command = True

    def timer_callback(self):
        msg = Bool()
        msg.data = self.command
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.command = not self.command

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GripperCommandNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

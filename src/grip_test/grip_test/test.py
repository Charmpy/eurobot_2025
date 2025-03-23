import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32
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

from std_srvs.srv import SetBool

from .util import Gripper

class TestNode(Node):
    def __init__(self):
        try:
            super().__init__('test_node')
            
            self.get_logger().info('Launched')
            
            self.create_subscription(Int32, '/test', self.callback, 10)

            self.gripper = Gripper(self)
            # self.gripper.grip_center(namel='cylinder12', namer='cylinder13')
            # self.gripper.grip_sides(namel='cylinder11', namer='cylinder14')
        except KeyboardInterrupt:
            pass
    def callback(self, msg):
        self.get_logger().info(f'Got {msg.data}')
        if msg.data == 0:
            self.gripper.grip_center(namel='cylinder12', namer='cylinder13')
        elif msg.data == 1:
            self.gripper.grip_sides(namel='cylinder11', namer='cylinder14')
        elif msg.data == 2:
            self.gripper.release_center()
        elif msg.data == 3:
            self.gripper.release_sides()
    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TestNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

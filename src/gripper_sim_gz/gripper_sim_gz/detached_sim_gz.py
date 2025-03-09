import rclpy
from rclpy.node import Node

from std_msgs.msg import String
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

class GripNode(Node):

    def __init__(self):
        try:
            
            super().__init__('grip_node')
            
            self.get_logger().info('Launched')
            
            self.client = gz_transport.Node()  # Создаём ноду Gazebo Transport
            self.set_model_speed = self.client.advertise('/joint_1', Double)  # Публикуем скорость

            timer_period = 1 
            self.timer = self.create_timer(timer_period, self.timer_callback)

            self.X = 0.0
            
            self.detach = self.client.advertise('/detach_topic', Boolean)
            self.attach = self.client.advertise('/attach_topic', Boolean)

            self.state = False
            self.create_service(SetBool, 'grip_node/change_state', self.change_state_callback)
        except KeyboardInterrupt:
            pass

    def change_state_callback(self, request, response):
        self.state = bool(request.data)

        # self.change_state.publish(msg)
        response.success = True
        response.message = f'Set state to {request.data}'


        msg = Boolean()
        msg.data = True

        self.attach.publish(msg) if self.state else self.detach.publish(msg)

        # self.change_state.publish(msg)
        self.get_logger().info(f'Set gripper state to {request.data}')
        return response

    def timer_callback(self):



        msg = Double()

        # msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = self.X * 0.1

        self.set_model_speed.publish(msg)

        if (self.X + 1)  * 0.1 <= 0.3:
            self.X += 1
        else:
            self.X = 0

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = GripNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

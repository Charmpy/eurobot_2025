import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped

from ros_gz_interfaces.srv import SetEntityPose
import subprocess

import gz.transport13 as gz_transport  # Gazebo Transport API
from ros_gz_interfaces.srv import SetEntityPose
from gz.msgs10.pose_pb2 import Pose as GzPose  # Сообщение Pose

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.header_pb2 import Header  # Заголовок сообщения
from gz.msgs.twist_pb2 import Twist as GZTwist
from gz.msgs.model_pb2 import Model

from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster





import math

class HWNode(Node):

    def __init__(self):
        super().__init__('hw_node')

        self.client = gz_transport.Node()  # Создаём ноду Gazebo Transport
        self.set_model_speed = self.client.advertise('/model/my_bot/cmd_vel', GZTwist)  # Публикуем скорость


        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.X = 1.0
        self.Y = -0.2
        self.W = 0.0

        self.vel_X = 0.0
        self.vel_Y = 0.0
        self.vel_W = 0.0

        self.vel_X_prev = 0
        self.vel_Y_prev = 0
        self.vel_W_prev = 0


        self.prev_time = self.get_clock().now()


    def listener_callback(self, msg):
        self.vel_X = msg.linear.x
        self.vel_Y = msg.linear.y
        self.vel_W = msg.angular.z

        # self.get_logger().info(f"{msg.linear.x} {msg.linear.y} {msg.angular.z}")
    
    def timer_callback(self):
        vel_x, vel_y, vel_w = self.vel_X, self.vel_Y, self.vel_W
        time_now = self.get_clock().now()

        d_time = time_now - self.prev_time
        # self.get_logger().info(f"{d_time}")
        self.W += vel_w * d_time.nanoseconds/ 10**9

        self.X += vel_x * d_time.nanoseconds / 10**9 * math.cos(self.W) + vel_y * d_time.nanoseconds/ 10**9 * math.sin(self.W)
        self.Y += vel_y * d_time.nanoseconds/ 10**9 * math.cos(self.W) + vel_x * d_time.nanoseconds / 10**9 * math.sin(self.W)
        

        # self.get_logger().info(f"{self.X} {self.Y} {self.W}")

        pos_x, pos_y, pos_w = self.X, self.Y, self.W

        quaternion = quaternion_from_euler(0, 0, pos_w)
        # self.get_logger().info(f"{quaternion}")

        msg = Odometry()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = pos_x
        msg.pose.pose.position.y = pos_y
        msg.pose.pose.position.z = 0.01
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2] 
        msg.pose.pose.orientation.w = quaternion[3]


        msg.twist.twist.linear.x = vel_x
        msg.twist.twist.linear.y = vel_y
        msg.twist.twist.linear.z = 0.01

        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = vel_w

        self.publisher_.publish(msg)

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = pos_x
        t.transform.translation.y = pos_y
        t.transform.translation.z = 0.01

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message

        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        model_msg = Model()
        model_msg.name = 'my_bot'  # Имя модели

        # Добавляем скорость (движение вперёд)
        twist = GZTwist()
        twist.linear.x = vel_x  # Линейная скорость
        twist.linear.y = vel_y
        twist.angular.z = vel_w  # Угловая скорость


        # Отправляем команду в Gazebo
        self.set_model_speed.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = HWNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
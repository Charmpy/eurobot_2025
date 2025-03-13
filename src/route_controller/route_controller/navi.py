
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
# import math
# import time # Time library
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
# from rclpy.duration import Duration # Handles time for ROS 2
# from .robot_navigator import BasicNavigator # Helper module



# Это класс для работы с нав2 из программы


class RobotUtil(Node):
    def __init__(self):
        super().__init__('driver')
        self.subscription1 = self.create_subscription(
            Int16, 'camera/marker_error',   
            self.listener_callback,
            10)
        self.subscription1 = self.create_subscription(
            Int64, 'camera/marker_size',   
            self.listener_size_callback,
            10)
        self.error = 0
        self.subscription1

        self.FLAG = False
        self.FLAG2 = False
        
        self.publisher_twist = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

    

    def send_request(self, command):
        self.req.command = command
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def start_request(self):
        ans = self.send_request("Start")
        self.get_logger().info('target ArUco: ' + ans.result)
    
    def begin_targeting(self):
        ans = self.send_request("Catch")
        self.get_logger().info('target ArUco: ' + ans.result)


    def listener_callback(self, msg):
        if self.FLAG:
            self.error = msg.data
            if abs(self.error) > 5:
                self.send_msg_rot()
                self.get_logger().info('ERROR ' + str(-self.error))
            else: 
                self.FLAG = False
                self.get_logger().info('ERROR Complete')
        
        else:
            self.get_logger().info('not start ERROR ' + str(-self.error))
        
    def listener_size_callback(self, msg):
        if self.FLAG2:
            self.error = msg.data
            if abs(self.error) < 11000:
                self.send_msg_for()
                self.get_logger().info('Size ' + str(-self.error))
            else: 
                self.FLAG2 = False
                self.get_logger().info('ERROR Complete')
                self.send_msg_stop()
        
        else:
            self.get_logger().info('not start ERROR ' + str(-self.error))
            

    def send_msg_rot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = -self.error * 0.005
        # self.get_logger().info('DELTA ' + str(self.error * 0.01))
        self.publisher_twist.publish(msg)
    
    def send_msg_for(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_twist.publish(msg)
    
    def send_msg_back(self):
        msg = Twist()
        msg.linear.x = -0.1
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_twist.publish(msg)
    
    
    def send_msg_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.publisher_twist.publish(msg)

    
    def publish(self, data):
        msg = String()
        msg.data = data
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
    

    def move_to_aruco(self):
        # self.begin_targeting()
        while abs(self.error) >= 5:
            rclpy.Rate(100).sleep()

            
        self.send_msg_stop()
        pass


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

    # Функция для указания стартового положения робота на карте

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


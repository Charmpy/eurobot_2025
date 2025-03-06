import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from .navi import Navi
import time
import math


from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from .robot_navigator import BasicNavigator # Helper module
from .util import Servo, Gripper, ServoControl
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist 
from req_res_str_service.srv import ReqRes


class CameraReq(Node):
    def __init__(self):
        super().__init__('camera_req')
        self.cli = self.create_client(ReqRes, 'cam_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ReqRes.Request()

    def send_request(self, a):
        self.req.req = a
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()



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


class RobotEsteminator:
    def __init__(self, direction):
        self.x = 0
        self.y = 2
        self.rot_index = direction
    
    def check(self):
        posabilities = []
        if self.rot_index == 0:
            if self.y != 2:
                posabilities.append('f')
            if self.x != 0:
                posabilities.append("r")
            if self.x != 2:
                posabilities.append("l")
        elif self.rot_index == 1:
            if self.x != 0:
                posabilities.append('f')
            if self.y != 0:
                posabilities.append("r")
            if self.y != 2:
                posabilities.append("l")
        elif self.rot_index == 2:
            if self.y != 0:
                posabilities.append('f')
            if self.x != 2:
                posabilities.append("r")
            if self.x != 0:
                posabilities.append("l")
        elif self.rot_index == 3:
            if self.x != 2:
                posabilities.append('f')
            if self.y != 2:
                posabilities.append("r")
            if self.y != 0:
                posabilities.append("l")
        return posabilities
    

    def move(self, dir):
        if dir in self.check():
            if self.rot_index == 0:
                self.y += 1
            elif self.rot_index == 1:
                self.x -= 1
            elif self.rot_index == 2:
                self.y -= 1
            elif self.rot_index == 3:
                self.x += 1

            if dir == "r":
                if self.rot_index == 3:
                    self.rot_index = 0
                else:
                    self.rot_index += 1
            elif dir == 'l':
                if self.rot_index == 0:
                    self.rot_index = 3
                else:
                    self.rot_index -= 1

            return True
        else:
            return False

    def get_coords(self):
            return ((2 - self.y) * 1.6, self.x * -1.6, (-self.rot_index * 1.57) - 3.14)

    def little_back(self):
        dx = 0
        dy = 0
        if self.rot_index == 0:
            dx = 0.05
        elif self.rot_index == 1:
            dy = +0.05
        elif self.rot_index == 2:
            dx = -0.05
        elif self.rot_index == 3:
            dy = -0.05
        return ((2 - self.y) * 1.6 + dx, self.x * -1.6 + dy, (-self.rot_index * 1.57) - 3.14)

        

    def get_str(self):
        a = [
            ["*", "*", "*"],
            ["*", "*", "*"],
            ["*", "*", "*"]
        ]
        d = {0: "D", 1:"L", 2:'F', 3:"R"}
        a[self.y][self.x] = d[self.rot_index]
        return ["".join(a[0]), "".join(a[1]), "".join(a[2])]


def main(args=None):
    rclpy.init(args=args)


    camera_req = CameraReq()

    navigator = BasicNavigator()

    navi = Navi()
    navi.publish(0.0, 0.0, 0.0)
    navigator.waitUntilNav2Active()

    RE = RobotEsteminator(2)

    ##### main circlue
    _, _, old_rot = RE.get_coords()
    response = camera_req.send_request("p").res
    ink = 0
    ink2 = 0
    
    while response != "a":
        if response != 'N':
            print(response)
            time_ = navigator.get_clock().now().to_msg()
            posible = RE.move(response)
            if posible:
                print(response)
                x, y, rot = RE.get_coords()
                ### first arrow
                goal_pose = Navi.set_goal_pose(x, y, old_rot, time_)
                navigator.goToPose(goal_pose)
                while not navigator.isNavComplete():
                    pass
                goal_pose = Navi.set_goal_pose(x, y, rot, time_)
                navigator.goToPose(goal_pose)
                while not navigator.isNavComplete():
                    pass
                print("\n".join(RE.get_str()))
                _, _, old_rot = RE.get_coords()    
        else:
            print("NO SIGN")
            time_ = navigator.get_clock().now().to_msg()
            x, y, rot = RE.get_coords()
            if ink % 4 == 0:
                goal_pose = Navi.set_goal_pose(x, y, old_rot + 3.14/16, time_)
            elif ink % 3 == 0:
                goal_pose = Navi.set_goal_pose(x, y, old_rot - 3.14/16, time_)
            elif ink % 2 == 0:
                goal_pose = Navi.set_goal_pose(x, y, old_rot, time_)
            ink += 1
 

            navigator.goToPose(goal_pose)
            
            while not navigator.isNavComplete():
                pass
            

        response = camera_req.send_request("p").res
    posible = RE.move('f')
    x, y, rot = RE.get_coords()
    ### first arrow
    goal_pose = Navi.set_goal_pose(x, y, old_rot, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    goal_pose = Navi.set_goal_pose(x, y, rot, time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass
    print("\n".join(RE.get_str()))  

    print("DONE")


    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
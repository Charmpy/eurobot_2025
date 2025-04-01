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
from .util import Gripper, RobotMacros
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist 
# from req_res_str_service.srv import ReqRes
from .navi import RobotUtil



# class CameraReq(Node):
#     def __init__(self):
#         super().__init__('camera_req')
#         self.cli = self.create_client(ReqRes, 'cam_service')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('service not available, waiting again...')
#         self.req = ReqRes.Request()

#     def send_request(self, a):
#         self.req.req = a
#         self.future = self.cli.call_async(self.req)
#         rclpy.spin_until_future_complete(self, self.future)
#         return self.future.result()


# Этот класс нужен для ориентирования робота на поле ртк, его не удаляю, чтобы не поломалось

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


# собсна главный цикл программы

def main(args=None):
    rclpy.init(args=args)
    RM = RobotMacros()
    RM.start_pose()

    RM.grip_cans(point=1)
    RM.move_up()
    RM.time_move(-0.3, 0.5)
    # time.sleep(10)

    navigator = BasicNavigator()

    navi = Navi()    
    navi.publish(1.0, -0.3, 0.111) # почему-то робот все равно появлялся в точке (0,0,0). Так что задаю initial_pose через nav2_params
    navigator.waitUntilNav2Active() 

    while True:
        time_ = navigator.get_clock().now().to_msg()
        
        # x,y,rot = (0.1, -1, -0.16)    
        x,y,rot = (2.0, -1, -0.16)           
        goal_pose = Navi.set_goal_pose(x, y, rot, time_)
        navigator.goToPose(goal_pose)
        while not navigator.isNavComplete():
            pass 



    # camera_req = CameraReq()

    # navigator = BasicNavigator()

    # navi = Navi()
    # navi.publish(0.0, 0.0, 0.0)
    # navigator.waitUntilNav2Active()

    # RE = RobotEsteminator(2)

    # ##### main circlue
    # _, _, old_rot = RE.get_coords()
    # response = camera_req.send_request("p").res
    # ink = 0
    # ink2 = 0
    
    # while response != "a":
    #     if response != 'N':
    #         print(response)
    #         time_ = navigator.get_clock().now().to_msg()
    #         posible = RE.move(response)
    #         if posible:
    #             print(response)
    #             x, y, rot = RE.get_coords()
    #             ### first arrow
    #             goal_pose = Navi.set_goal_pose(x, y, old_rot, time_)
    #             navigator.goToPose(goal_pose)
    #             while not navigator.isNavComplete():
    #                 pass
    #             goal_pose = Navi.set_goal_pose(x, y, rot, time_)
    #             navigator.goToPose(goal_pose)
    #             while not navigator.isNavComplete():
    #                 pass
    #             print("\n".join(RE.get_str()))
    #             _, _, old_rot = RE.get_coords()    
    #     else:
    #         print("NO SIGN")
    #         time_ = navigator.get_clock().now().to_msg()
    #         x, y, rot = RE.get_coords()
    #         if ink % 4 == 0:
    #             goal_pose = Navi.set_goal_pose(x, y, old_rot + 3.14/16, time_)
    #         elif ink % 3 == 0:
    #             goal_pose = Navi.set_goal_pose(x, y, old_rot - 3.14/16, time_)
    #         elif ink % 2 == 0:
    #             goal_pose = Navi.set_goal_pose(x, y, old_rot, time_)
    #         ink += 1
 

    #         navigator.goToPose(goal_pose)
            
    #         while not navigator.isNavComplete():
    #             pass
            

    #     response = camera_req.send_request("p").res
    # posible = RE.move('f')
    # x, y, rot = RE.get_coords()
    # ### first arrow
    # goal_pose = Navi.set_goal_pose(x, y, old_rot, time_)
    # navigator.goToPose(goal_pose)
    # while not navigator.isNavComplete():
    #     pass
    # goal_pose = Navi.set_goal_pose(x, y, rot, time_)
    # navigator.goToPose(goal_pose)
    # while not navigator.isNavComplete():
    #     pass
    # print("\n".join(RE.get_str()))  

    print("DONE")


    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
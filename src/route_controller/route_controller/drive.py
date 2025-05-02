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
from .coord_handler import CoordHandler as CH
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist 
# from req_res_str_service.srv import ReqRes
from .navi import RobotUtil

def main(args=None):
    rclpy.init(args=args)
    RM = RobotMacros()
    # time.sleep(0.1)
    # RM.com_start_state()
    # # time.sleep(0.1)
    # # RM.time_move(0.05, 2)
    # time.sleep(0.1)
    # RM.com_compile()
    # time.sleep(0.1)
    # RM.time_move(0.05, 0.5)
    # time.sleep(0.1)
    # RM.com_build()
    # time.sleep(0.1)
    # RM.time_move(-0.1, 2)
    # time.sleep(0.1)
    # RM.com_start_state()
    # time.sleep(0.1)

    navigator = BasicNavigator()
    coords = CH()

    # navigator = Navi()    
    # navigator.configure_init_pose(1.0, -0.3, 0.111)

    time.sleep(0.1)
    RM.com_start_state()

    while True:
        time_ = navigator.get_clock().now().to_msg()
            
        # goal_pose = Navi.set_goal_pose(*CH.point_1(), time_)
        goal_pose = Navi.set_goal_pose(coords.get_storage())
        navigator.goToPose(goal_pose)
        while not navigator.isNavComplete():
            continue

        goal_pose = Navi.set_goal_pose(coords.get_goal())

        navigator.goToPose(goal_pose)
        while not navigator.isNavComplete():
            continue

        RM.com_compile()
        RM.com_build()


    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
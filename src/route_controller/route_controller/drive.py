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
    RM.start_pose()

    RM.grip_cans(point=1)
    RM.move_up()
    RM.time_move(-0.3, 0.5)
    # time.sleep(10)

    navigator = BasicNavigator()

    navi = Navi()    
    navi.publish(1.0, -0.3, 0.111) # почему-то робот все равно появлялся в точке (0,0,0). Так что задаю initial_pose через nav2_params
    navigator.waitUntilNav2Active() 

    # while True:
    time_ = navigator.get_clock().now().to_msg()
        
    goal_pose = Navi.set_goal_pose(*CH.point_1(), time_)
    navigator.goToPose(goal_pose)
    while not navigator.isNavComplete():
        pass 

    print("DONE")


    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from .navi import Navi
import time
import math


from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from .robot_navigator import BasicNavigator, NavigationResult # Helper module
from .util import Gripper, RobotMacros
from .coord_handler import CoordHandler 
from std_msgs.msg import Int16
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist 
# from req_res_str_service.srv import ReqRes
from .navi import RobotUtil

# import gpiod

# sudo apt istall libgpiod-dev python3-libgpiod




class StartListener(Node):
    def __init__(self):
        super().__init__('start_listener')
        self.subscription = self.create_subscription(
            String,
            'start_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.start = False

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if msg.data == 'start': 
            self.start = True
    
    def get_start_callback(self):
        return self.start


def go_to_goal(navigator, coords, goal_type):
    while True:
        time_ = navigator.get_clock().now().to_msg()        

        if goal := coords.get_goal(goal_type):  
            print(goal)          
            goal_pose = Navi.set_goal_pose(*goal, time_)
        else:
            return False

        navigator.goToPose(goal_pose)
        if navigator.getResult() != NavigationResult.FAILED:
            coords.add_goal(goal, goal_type)
            break

    return True

    


def main(args=None):
    rclpy.init(args=args)
    logger = rclpy.logging.get_logger('aboba')
    rclpy.logging.set_logger_level('aboba', rclpy.logging.LoggingSeverity.DEBUG)
    logger.debug("Start")

    RM = RobotMacros()
    RM.com_start_state()
    logger.debug("First start state, WAIT FOR KEY")

    start_listener = StartListener()

    COLOR_FLAG = 'Y'   # "B"
    SAFETY_FLAG = True

    while True:
        rclpy.spin_once(start_listener)
        if start_listener.get_start_callback():
            break
        time.sleep(0.1)

    time.sleep(0.1)
    # RM.com_start_state()
    # time.sleep(0.1)
    # RM.time_move(0.1, 5)
    # time.sleep(0.1)
    # RM.time_move(-0.1, 5)
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

    # ======= Wotking miniprogramm ===========
    # time.sleep(0.5)
    # RM.place_flag()
    # logger.debug("поставил флаг")
    # time.sleep(0.5)

    # RM.time_move(0.05, 5)
    # time.sleep(0.5)
    # # RM.time_move(-0.1, 5)

    # logger.debug("собираю")
    # RM.com_compile()
    # time.sleep(2.5)
    # logger.debug("собрал")
    # RM.time_move(-0.05, 2)
    # time.sleep(0.1)

    # logger.debug("строю")
    # RM.com_build()
    # time.sleep(2)    
    # logger.debug("построил")
    # RM.time_move(-0.05, 4)
    # time.sleep(0.1)

    # logger.debug("Last start state")
    # RM.com_start_state()
    # time.sleep(0.5)
    # ======= Wotking miniprogramm ===========

    

    navigator = BasicNavigator()
    # coords = CoordHandler()

    # all_storages_complete = False
    # all_goals_complete = False

    time.sleep(1)
    while True:
        logger.debug("Я в цикле")


        ## подъезд к трибуне
        time_ = navigator.get_clock().now().to_msg()
        goal_pose = Navi.set_goal_pose(0.65, -1.04, 0, time_)
        logger.debug("Я поехал 1")
        navigator.goToPose(goal_pose)
        while not navigator.isNavComplete():
            # if navigator.getResult() != NavigationResult.FAILED:
            #     logger.debug("HUUUUUIII")
            #     break
            continue
        logger.debug("Я приехал 1")

        ## проезжаем немного вперед
        time_ = navigator.get_clock().now().to_msg()
        goal_pose = Navi.set_goal_pose(0.80, -1.04, 0, time_)
        logger.debug("Я поехал 2")
        # RM.com_position()
        navigator.goToPose(goal_pose)
        while not navigator.isNavComplete():
            continue
        logger.debug("Я приехал 2")

        ## собираем трибуну
        logger.debug("собираю")
        RM.com_compile()
        time.sleep(2.5)
        logger.debug("собрал")


        # ## разворачиваемся
        # time_ = navigator.get_clock().now().to_msg()
        # goal_pose = Navi.set_goal_pose(0.80, -1.05, 180, time_)
        # logger.debug("Я разворачиваюсь")
        # navigator.goToPose(goal_pose)
        # while not navigator.isNavComplete():
        #     continue
        # logger.debug("Я развернулся")

        # едем с банками в зону
        time_ = navigator.get_clock().now().to_msg()
        goal_pose = Navi.set_goal_pose(0.25, -1.25, 180, time_)
        logger.debug("Я поехал 3")
        # navigator.goToPose(goal_pose)
        # while not navigator.isNavComplete():
        #     continue
        logger.debug("Я приехал 3")
        RM.com_build()
        time.sleep(4)

        RM.time_move(-0.1, 5)
        RM.com_start_state()

        break
    # navigator = Navi()    
    # # navigator.configure_init_pose(1.0, -0.3, 0.111)
    # # while True:
    # #     time.sleep(0.1)
    # #     RM.com_start_state()
    # #     time.sleep(2.0)
    # #     logger.debug("Start")
    # # go_to_goal(H)
    # #     RM.time_move(-0.1,1.0)
    # #     time.sleep(5.0)
    # while not (all_storages_complete or all_goals_complete):
    #     goal = set_goal_pose(navigator, coords, goal_type)
    # goal_pose = Nav.set_goal_pose
    #     # goal_type = "storage"
    #     # if not go_to_goal(navigator, coords, goal_type):
    #     #     all_storages_complete = True
    #     #     print("all_storages_complete")

    #     logger.debug("Я еду")
    #     while not navigator.isNavComplete():
    #         continue

    #     logger.debug("Я подъезжаю")
    #     RM.com_position()
    #     logger.debug("Я собираю")
    #     time.sleep(10)
    #     # RM.com_compile()

    #     goal_type = "blue"
    #     if not go_to_goal(navigator, coords, goal_type):
    #         all_goals_complete = True 
    #         print("all_goals_complete")     

    #     logger.debug("Я еду")
    #     while not navigator.isNavComplete():
    #         continue         

    #     # logger.debug("Я строю")
    #     # RM.com_build()
    #     # logger.debug("Я долбоеб")
    #     RM.com_start_state()

    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
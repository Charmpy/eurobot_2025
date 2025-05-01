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


def main(args=None):
    rclpy.init(args=args)
    RM = RobotMacros()
    time.sleep(0.1)
    start_listener = StartListener()

    while True:
        rclpy.spin_once(start_listener)
        if start_listener.get_start_callback():
            break
        time.sleep(0.1)


    # BUTTON_PIN = 27
    # chip = gpiod.Chip('gpiochip4')
    # button_line = chip.get_line(BUTTON_PIN)

    # button_line.request(consumer="Button", type=gpiod.LINE_REQ_DIR_IN)

    # try:
    #     while True:
    #         button_state = button_line.get_value()
    #         if button_state == 1:  # Button is pressed
    #             break
    # finally:
    #     button_line.release()

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

    RM.place_flag()
    time.sleep(0.1)

    # navigator = BasicNavigator()

    # navi = Navi()    
    # navi.publish(1.0, -0.3, 0.111) # почему-то робот все равно появлялся в точке (0,0,0). Так что задаю initial_pose через nav2_params
    # navigator.waitUntilNav2Active() 

    # # while True:
    # time_ = navigator.get_clock().now().to_msg()
        
    # goal_pose = Navi.set_goal_pose(*CH.point_1(), time_)
    # navigator.goToPose(goal_pose)
    # while not navigator.isNavComplete():
    #     pass 

    print("DONE")


    rclpy.shutdown()

    

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time
import math


from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from robot_lifters import BoardLiftNode, CanLiftNode, CanRotator1Node, CanRotator2Node

from std_msgs.msg import Int16
from std_msgs.msg import Int64
from geometry_msgs.msg import Twist 

class Robot:
    BOARD_LIFT_MIN = 0
    BOARD_LIFT_MAX = 0.1
    CAN_LIFT_MIN = 0
    CAN_LIFT_MAX = 0.2
    CAN_ROTATION_MIN = 0
    CAN_ROTATION_MAX = 0.1

    def __init__(self):
        self.board_lift_pos = 0
        self.can_lift_pos = 0
        self.can_rotator_1_pos = 0
        self.can_rotator_2_pos = 0

    def board_lift_move(self, direction):
        if(direction == "up"):
            self.board_lift_pos = self.BOARD_LIFT_MAX
        elif(direction == "down"):
            self.board_lift_pos = self.BOARD_LIFT_MIN
    
    def can_lift_move(self, direction):
        if(direction == "up"):
            self.can_lift_pos = self.CAN_LIFT_MAX
        elif(direction == "down"):
            self.can_lift_pos = self.CAN_LIFT_MIN

    def right_can_rotator_move(self, direction):
        if(direction == "in"):
            self.can_rotator_1_pos = self.CAN_ROTATION_MIN
        elif(direction == "out"):
            self.can_rotator_1_pos = self.CAN_ROTATION_MAX

    def left_can_rotator_move(self, direction):
        if(direction == "in"):
            self.can_rotator_2_pos = self.CAN_ROTATION_MIN
        elif(direction == "out"):
            self.can_rotator_2_pos = self.CAN_ROTATION_MAX

    def get_joint_pos(self, index):
        if(index == "board"):
            return (self.board_lift_pos)
        if(index == "can_lift"):
            return (self.can_lift_pos)
        if(index == "can_1_angle"):
            return (self.can_rotator_1_pos)
        if(index == "can_2_angle"):
            return (self.can_rotator_2_pos)
        

def main(args=None):
    rclpy.init(args=args)
    
    RE = Robot()

    board_lift = BoardLiftNode()
    can_lift = CanLiftNode()
    can_1_rot = CanRotator1Node()
    can_2_rot = CanRotator2Node()

    while(1):
        RE.board_lift_move("up")
        RE.can_lift_move("up")
        RE.right_can_rotator_move("out")
        RE.left_can_rotator_move("out")

        board_lift.publish(RE.get_joint_pos("board"))
        can_lift.publish(RE.get_joint_pos("can_lift"))
        can_1_rot.publish(RE.get_joint_pos("can_1_angle"))
        can_2_rot.publish(RE.get_joint_pos("can_2_angle"))

        time.sleep(5)

        RE.board_lift_move("down")
        RE.can_lift_move("down")
        RE.right_can_rotator_move("in")
        RE.left_can_rotator_move("in")

        board_lift.publish(RE.get_joint_pos("board"))
        can_lift.publish(RE.get_joint_pos("can_lift"))
        can_1_rot.publish(RE.get_joint_pos("can_1_angle"))
        can_2_rot.publish(RE.get_joint_pos("can_2_angle"))

        time.sleep(5)

if __name__ == '__main__':
    main()
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Header


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_pub')
        self.declare_parameter('frame_id', '')
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1  # seconds
        self.i = 0
        
        self.timer = self.create_timer(timer_period, self.listener_callback)
        # self.subscription  # prevent unused variable warning

        print('Started twist_stamper!')

    def listener_callback(self):
        msg = Twist()
        if self.i % 2 == 0:
            msg.linear.x = 0.3
        else:
            msg.linear.x = -0.3
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.linear.x)
        self.i += 1
        # self.get_logger().info('X: "%f"' % outMsg.twist.linear.x)


def main(args=None):
    rclpy.init(args=args)

    twist_stamper = TwistStamper()
    try:
        rclpy.spin(twist_stamper)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')

    print('Exiting...')

    twist_stamper.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
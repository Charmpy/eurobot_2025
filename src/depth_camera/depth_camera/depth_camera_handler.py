import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

class BoardDetector(Node):
    def __init__(self):
        super().__init__('board_detector')
        self.bridge = CvBridge()
        # подключение к топику камеры
        self.depth_sub = self.create_subscription(Image, 'depth_left', self.depth_callback, 10)

        self.start_positioning_sub = self.create_subscription(String, 'positioning', self.position_calback, 10)
        self.positioning_pub = self.create_publisher(String, 'positioning', 10)
        # self.image_pub = self.create_publisher(Image, 'image_topic_2', 10)

        # топик с изображением для отладки
        self.image_pub = self.create_publisher(Image, 'image_topic_2', 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.prev_time = self.get_clock().now()
        self.integral = 0
        self.x_integral = 0
        self.y_integral = 0

        self.x_done = True
        self.y_done = True
        self.w_done = True

        self.trouble_counter = 0

    def position_calback(self, msg):
        if msg.data == "start":
            self.integral = 0
            self.x_integral = 0
            self.y_integral = 0
            self.start_time = self.get_clock().now().nanoseconds * (10^-9)
            self.x_done = False
            self.y_done = False
            self.w_done = False
        if msg.data == "stop":
            self.x_done = True
            self.y_done = True
            self.w_done = True


    def depth_callback(self, msg):
        # преобразование в читаемый cv формат
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        kernel = np.ones((3,3),np.float32)/25
        blured = cv.filter2D(depth_image,-1,kernel)

        # заменяем бесконечности на максимальное расстояние
        depth_clipped = np.nan_to_num(depth_image, neginf=255, posinf=255) 

        # в 32FC1 значение каждого пикселя это расстояние до объекта в метрах, меняем в сантиметры
        # depth_clipped = depth_clipped * 100
        # преобразуем в юинт8 для работы с методами cv
        depth_clipped = depth_clipped.astype(np.uint8)
        # инвертируем для удобства работы
        depth_clipped = cv.bitwise_not(depth_clipped)
        # с ограничением с помощью дросселя плохо заработало
        ret,thresh4 = cv.threshold(depth_clipped,170,255,cv.THRESH_BINARY)
        thresh4[:, -1] = 0


        kernel = np.ones((7,7),np.uint8)
        erosion = cv.erode(thresh4,kernel,iterations = 1)

        opening = cv.morphologyEx(thresh4, cv.MORPH_OPEN, kernel)
        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(erosion))


        contours, hierarchy = cv.findContours(erosion, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            if self.trouble_counter < 5:
                self.trouble_counter += 1
            elif self.trouble_counter == 5:
                print("x5")
                msg = String()
                msg.data = "error"
                self.positioning_pub.publish(msg)
                msg = Twist()
                self.publisher_.publish(msg)
                self.trouble_counter += 1
            return
        cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
        cnt = cntsSorted[-1]
        M = cv.moments(cnt)
        if M['m00'] == 0:
            return
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        # print(cx, cy)
        # ищем контуры
        edges = cv.Canny(depth_clipped, 20,20)
        # для отладки
        cdst = cv.cvtColor(depth_clipped, cv.COLOR_GRAY2BGR)
        # cv.drawContours(cdst, contours, 0, (0,255,0), 3)
        cv.circle(cdst,(cx, cy), 2, (0,255,255), -1)

        peri = cv.arcLength(cnt, True)

        eps = 0.1
        f_1 = False
        f_2 = False
        f_3 = False
        f_4 = False
        approx = cv.approxPolyDP(cnt, eps * peri, True)
        cv.drawContours(cdst,[approx],0,(0,0,255),2)
        if len(approx) == 4:
            for i in approx:
                if i[0][0] > cx and i[0][1] < cy and not f_1:
                    # print(i)
                    r_u = i
                    cv.circle(cdst, r_u[0], 5, (0,255,255), -1)
                    f_1 = True
                
                if i[0][0] > cx and i[0][1] > cy and not f_2:
                    # print(i)
                    r_d = i
                    cv.circle(cdst, r_d[0], 5, (0,255,255), -1)
                    f_2 = True
                if i[0][0] < cx and i[0][1] > cy and not f_3:
                    # print(i)
                    l_d = i
                    cv.circle(cdst, l_d[0], 5, (0,255,255), -1)
                    f_3 = True
                if i[0][0] < cx and i[0][1] < cy and not f_4:
                    # print(i)
                    l_u = i
                    cv.circle(cdst, l_u[0], 5, (0,255,255), -1)
                    f_4 = True
            if not f_1 or not f_2 or not f_3 or not f_4:
                return

            dx = r_u[0][0] - l_u[0][0]
            dy = r_u[0][1] - l_u[0][1]
            
            self.angle_error = -2.73 - math.degrees(math.atan(dy/dx))
            self.y_error = - l_u[0][0] + 14
            self.x_error = - l_u[0][1] + 26
            # print(self.error)
            # print(self.y_error, self.x_error)
        else:
            return
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cdst))


        if not (self.x_done and self.y_done and self.w_done):
            if not self.w_done:
                self.angle_control()
            self.linear_control()
            print("---")


    def linear_control(self):
        ki = 0.001
        kp = 0.005
        dt = self.get_clock().now().nanoseconds - self.prev_time.nanoseconds
        dt = dt * (10**-9)

        if self.w_done: 
            if abs(self.x_error) > 1:
                self.x_integral += (dt * self.x_error)
                if self.x_integral * ki  > 0.05:
                    self.x_integral = 0.05 / ki
                
                msg = Twist()
                msg.linear.x = self.x_error * kp + self.x_integral * ki
                print(f'x упр: {self.x_error * kp + self.x_integral * ki}')
                self.publisher_.publish(msg)
            else:
                if not self.x_done:
                    msg = Twist()
                    self.publisher_.publish(msg)
                    self.x_done = True
                    print("done x")
                    self.x_integral = 0

            if abs(self.y_error) > 3:
                self.y_integral += (dt * self.y_error)
                if self.y_integral * ki  > 0.05:
                    self.y_integral = 0.05 / ki
                
                msg = Twist()
                msg.linear.y = self.y_error * kp + self.y_integral * ki
                print(f'y упр: {self.y_error * kp + self.y_integral * ki}')
                self.publisher_.publish(msg)
            else:
                if not self.y_done:
                    msg = Twist()
                    print("done y")
                    self.publisher_.publish(msg)
                    self.y_done = True
                    self.y_integral = 0
            self.prev_time = self.get_clock().now()

        
    def angle_control(self):
        ki = 0.005
        kp = 0.02
        dt = self.get_clock().now().nanoseconds - self.prev_time.nanoseconds
        dt = dt * (10**-9)

        if abs(self.angle_error) > 2:
            print(f'w err: {self.angle_error}')
            self.integral += (dt * self.angle_error)
            if self.integral > 0.2 / kp:
                self.integral = 0.2 / kp

            w_con = self.angle_error * kp + self.integral * ki
            if w_con > 0.5:
                w_con = 0.5
            msg = Twist()
            msg.angular.z = w_con
            print(f'w упр: {w_con}')
            self.publisher_.publish(msg)
        else:
            if not self.w_done:
                msg = Twist()
                print("done")
                self.publisher_.publish(msg)
                self.integral = 0
                self.w_done = True
        self.prev_time = self.get_clock().now()


def main(args=None):
    rclpy.init(args=args)
    node = BoardDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

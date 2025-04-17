import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import math

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped

class BoardDetector(Node):
    def __init__(self):
        super().__init__('board_detector')
        self.bridge = CvBridge()
        # подключение к топику камеры
        self.depth_sub = self.create_subscription(Image, '/robot_camera/depth_image', self.depth_callback, 10)
        # топик с изображением для отладки
        self.image_pub = self.create_publisher(Image, 'image_topic_2', 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.prev_time = self.get_clock().now()
        self.integral = 0
        self.x_integral = 0
        self.y_integral = 0

        self.x_done = False
        self.y_done = False
        self.w_done = False

    def depth_callback(self, msg):
        # преобразование в читаемый cv формат
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        # заменяем бесконечности на максимальное расстояние
        depth_clipped = np.nan_to_num(depth_image, neginf=2.55, posinf=2.55) 

        # в 32FC1 значение каждого пикселя это расстояние до объекта в метрах, меняем в сантиметры
        depth_clipped = depth_clipped * 100
        # преобразуем в юинт8 для работы с методами cv
        depth_clipped = depth_clipped.astype(np.uint8)
        # инвертируем для удобства работы
        depth_clipped = cv.bitwise_not(depth_clipped)
        # с ограничением с помощью дросселя плохо заработало
        ret,thresh4 = cv.threshold(depth_clipped,230,255,cv.THRESH_BINARY)
        thresh4[:, -1] = 0


        contours, hierarchy = cv.findContours(thresh4, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        cnt = contours[0]
        M = cv.moments(cnt)
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
            # cv.circle(cdst,*lu, 3, (255,0,255), -1)
            # cv.circle(cdst, *ld, 3, (0,255,255), -1)
            # print(approx)


        # ищем линии
        # lines = cv.HoughLines(edges, 1, np.pi / 180, 40, None, 0, 0)
        # # отрисовываем
        # line_1 = None
        # line_2 = None
        # if lines is not None:
        #     for i in range(0, len(lines)):
        #         rho = lines[i][0][0]
        #         theta = lines[i][0][1]
        #         #  убираем что не интерсено
        #         if math.degrees(theta) > 70 and math.sin(theta) * rho < 70:
        #             a = math.cos(theta)
        #             b = math.sin(theta)
        #             x0 = a * rho
        #             y0 = b * rho
        #             print(x0, y0)
        #             pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        #             pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #             cv.line(cdst, pt1, pt2, (0,0,255), 1, cv.LINE_AA)
        #             cv.circle(cdst,(int(x0),int(y0)), 2, (0,255,255), -1)
        #     print("---")

        # delta_1 = 0
        # delta_2 = 0

        # if line_1:
        #     a = math.cos(line_1[1])
        #     b = math.sin(line_1[1])
        #     x0 = a * line_1[0]
        #     y0 = b * line_1[0]
        #     # print(x0, y0)
        #     pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        #     pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #     cv.circle(cdst,(int(x0),int(y0)), 2, (255,0,255), -1)

        #     cv.line(cdst, pt1, pt2, (0,0,255), 1, cv.LINE_AA)
        #     # print(61 - math.degrees(line_1[1]))
        #     delta_1 = 61 - math.degrees(line_1[1])
        # if line_2:
        #     a = math.cos(line_2[1])
        #     b = math.sin(line_2[1])
        #     x0 = a * line_2[0]
        #     y0 = b * line_2[0]
        #     # print(x0, y0)
        #     pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
        #     pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
        #     cv.line(cdst, pt1, pt2, (0,0,255), 1, cv.LINE_AA)
        #     cv.circle(cdst,(int(x0),int(y0)), 2, (0,255,255), -1)
        #     # print(44 - math.degrees(line_2[1]))
        #     delta_2 = 44 - math.degrees(line_2[1])
        
        # self.error = delta_1 + delta_2
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cdst))
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
        ki = 0.02
        kp = 0.03
        dt = self.get_clock().now().nanoseconds - self.prev_time.nanoseconds
        dt = dt * (10**-9)

        if abs(self.angle_error) > 2:
            print(f'w err: {self.angle_error}')
            self.integral += (dt * self.angle_error)
            if self.integral > 0.3 / kp:
                self.integral = 0.3 / kp
            msg = Twist()
            msg.angular.z = self.angle_error * kp + self.integral * ki
            print(f'w упр: {self.angle_error * kp + self.integral * ki}')
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

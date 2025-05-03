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
        self.depth_sub_left = self.create_subscription(Image, '/depth_left', self.depth_callback_left, 10)
        self.depth_sub_right = self.create_subscription(Image, '/depth_right', self.depth_callback_right, 10)

        self.start_positioning_sub = self.create_subscription(String, 'positioning', self.position_calback, 10)
        self.positioning_pub = self.create_publisher(String, 'positioning', 10)
        # self.image_pub = self.create_publisher(Image, 'image_topic_2', 10)

        # топик с изображением для отладки
        self.image_pub_right = self.create_publisher(Image, 'cam_image_right', 10)
        self.image_pub_left = self.create_publisher(Image, 'cam_image_left', 10)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.prev_time = self.get_clock().now()
        self.integral = 0
        self.x_integral = 0
        self.y_integral = 0

        self.x_done = True
        self.y_done = True
        self.w_done = True
        self.choose_algoritm = "stop"

        self.trouble_counter = 0
        
        self.x_error_left = 0
        self.y_error_left = 0
        self.angle_error_left = 0
        self.x_error_right = 0
        self.y_error_right = 0
        self.angle_error_right = 0

        self.binar_min = 177
        


    def position_calback(self, msg):
        if msg.data == "right" or msg.data == "left" or msg.data == "both" or msg.data == "only_left" or msg.data == "only_right":
            self.integral = 0
            self.x_integral = 0
            self.y_integral = 0
            self.start_time = self.get_clock().now().nanoseconds * (10^-9)
            self.x_done = False
            self.y_done = False
            self.w_done = False
            self.choose_algoritm = msg.data
            self.stop_time = self.get_clock().now()
        
        if msg.data == "stop":
            self.x_done = True
            self.y_done = True
            self.w_done = True
            self.choose_algoritm = "stop"



    def depth_callback_left(self, msg):
        if self.choose_algoritm == "only_right":
            return
        # преобразование в читаемый cv формат
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        depth_image = cv.rotate(depth_image, cv.ROTATE_180)

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
        ret,thresh4 = cv.threshold(depth_clipped,self.binar_min,255,cv.THRESH_BINARY)
        thresh4[:, -1] = 0


        kernel = np.ones((7,7),np.uint8)
        erosion = cv.erode(thresh4,kernel,iterations = 1)

        opening = cv.morphologyEx(thresh4, cv.MORPH_OPEN, kernel)
        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(opening))

        # Поиск контуров деревяшки
        contours, hierarchy = cv.findContours(thresh4, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        #print(contours)
        if len(contours) == 0:
            #Левая не отвечает, ориентируемся по правой
            self.choose_algoritm = "right"
            # if self.trouble_counter < 5:
            #     self.trouble_counter += 1
            # elif self.trouble_counter == 5:
            #     print("Нет картинка 5 кадров подряд")
            #     msg = String()
            #     msg.data = "error"
            #     self.positioning_pub.publish(msg)
            #     msg = Twist()
            #     self.publisher_.publish(msg)
            #     self.trouble_counter += 1
            print("no contour")
            return
        #self.trouble_counter = 0


        cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
        cnt = cntsSorted[-1]
        M = cv.moments(cnt)
        if M['m00']!=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            cx = 0
            cy =0
        # print(cx, cy)

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

        # аппроксимация
        approx = cv.approxPolyDP(cnt, eps * peri, True)
        cv.drawContours(cdst,[approx],0,(0,0,255),2)
        if len(approx) == 4:
            # раскидование углов по положениям
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

            # Ориентируемся по обеим камерам
            if(self.choose_algoritm == "right"):
                self.choose_algoritm = "both"

            # Расчет ошибки
            dx = r_u[0][0] - l_u[0][0]
            dy = r_u[0][1] - l_u[0][1]
            #print("left: ",r_u[0][0], l_u[0][0], r_u[0][1], l_u[0][1])
            
            self.angle_error_left = -2.73 - math.degrees(math.atan(dy/dx)) - 7.42
            self.y_error_left = - l_u[0][0] + 14 + 17
            self.x_error_left = - l_u[0][1] + 26 + 16
            #print("errors")

            self.trouble_counter = 0
            # print(self.error)
            # print("left w, y, x: ",self.angle_error_left, self.y_error_left, self.x_error_left)

        else:
            #print("problem")
            if self.trouble_counter < 5:
                self.trouble_counter += 1
            elif self.trouble_counter == 5:
                # Левая не отвечает, ориентируемся по правой
                self.choose_algoritm = "right"
                print("левая: Нет доски 5 кадров подряд")
                msg = String()
                msg.data = "error"
                self.positioning_pub.publish(msg)
                msg = Twist()
                self.publisher_.publish(msg)
                self.trouble_counter += 1
                return
        self.image_pub_left.publish(self.bridge.cv2_to_imgmsg(cdst))

        if not (self.x_done and self.y_done and self.w_done):
            if not self.w_done:
                self.angle_control()
            self.linear_control()
            print("---")



    def depth_callback_right(self, msg):
        if self.choose_algoritm == "only_left":
            return
        # преобразование в читаемый cv формат
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        depth_image = cv.rotate(depth_image, cv.ROTATE_180)

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
        ret,thresh4 = cv.threshold(depth_clipped,self.binar_min,255,cv.THRESH_BINARY)
        thresh4[:, -1] = 0


        kernel = np.ones((7,7),np.uint8)
        erosion = cv.erode(thresh4,kernel,iterations = 1)

        opening = cv.morphologyEx(thresh4, cv.MORPH_OPEN, kernel)
        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(opening))

        # Поиск контуров деревяшки
        contours, hierarchy = cv.findContours(thresh4, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            # if self.trouble_counter < 5:
            #     self.trouble_counter += 1
            # elif self.trouble_counter == 5:
            #     print("Нет картинка 5 кадров подряд")
            #     msg = String()
            #     msg.data = "error"
            #     self.positioning_pub.publish(msg)
            #     msg = Twist()
            #     self.publisher_.publish(msg)
            #     self.trouble_counter += 1
            print("no contours")
            return
        #self.trouble_counter = 0

        cntsSorted = sorted(contours, key=lambda x: cv.contourArea(x))
        cnt = cntsSorted[-1]
        M = cv.moments(cnt)
        if M['m00']!=0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        else:
            cx = 0
            cy =0
        # print(cx, cy)

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

        # аппроксимация
        approx = cv.approxPolyDP(cnt, eps * peri, True)
        cv.drawContours(cdst,[approx],0,(0,0,255),2)
        if len(approx) == 4:
            # раскидование углов по положениям
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

            # Ориентируемся по обеим камерам
            if(self.choose_algoritm == "left"):
                self.choose_algoritm = "both"

            # Расчет ошибки
            dx = r_u[0][0] - l_u[0][0]
            dy = -(r_u[0][1] - l_u[0][1])
            #print("right: ",r_u[0][0], l_u[0][0], r_u[0][1], l_u[0][1])
            
            self.angle_error_right = 3.04 + math.degrees(math.atan(dy/dx)) + 8.62
            self.y_error_right = - r_u[0][0] + 94 - 31
            self.x_error_right = - r_u[0][1] + 26 + 15

            self.trouble_counter = 0
            # print(self.error)
            #print("right w, y, x: ",self.angle_error_right, self.y_error_right, self.x_error_right)
        else:
            if self.trouble_counter < 5:
                self.trouble_counter += 1
            elif self.trouble_counter == 5:                    
                # Правая не отвечает, ориентируемся по левой
                self.choose_algoritm = "left"
                print("правая: Нет доски 5 кадров подряд")
                msg = String()
                msg.data = "error"
                self.positioning_pub.publish(msg)
                msg = Twist()
                self.publisher_.publish(msg)
                self.trouble_counter += 1
                return
        self.image_pub_right.publish(self.bridge.cv2_to_imgmsg(cdst))

        if not (self.x_done and self.y_done and self.w_done):
            if not self.w_done:
                self.angle_control()
            self.linear_control()
            print("---")



    # Отработка линейной ошибки    
    def linear_control(self):
        if(self.choose_algoritm == "left" or self.choose_algoritm == "only_left"):
            self.x_error = self.x_error_left
            self.y_error = self.y_error_left
        elif(self.choose_algoritm == "right" or self.choose_algoritm == "only_right"):
            self.x_error = self.x_error_right
            self.y_error = self.y_error_right
        else:#if(self.choose_algoritm == "both"):
            self.x_error = (self.x_error_left + self.x_error_right)/2
            self.y_error = (self.y_error_left + self.y_error_right)/2


        print("w, y, x: ",self.angle_error, self.y_error, self.x_error)
        print(self.choose_algoritm)


        # Коэффициенты регулятора
        ki_x = 0.003
        kp_x = 0.0002
        ki_y = 0.002
        kp_y = 0.0002
        dt = self.get_clock().now().nanoseconds - self.prev_time.nanoseconds
        dt = dt * (10**-9)

        #Ограничение по времени 10с
        if (self.get_clock().now().nanoseconds*(10**-9) - self.stop_time.nanoseconds*(10**-9) > 10):
                msg = Twist()
                print("time is up")
                self.publisher_.publish(msg)
                self.integral = 0
                self.y_done = True
                self.x_done = True
                self.choose_algoritm = "stop"

        # Начало работы линейной после угловой    
        if self.w_done:
            if abs(self.y_error) > 3:
                self.y_integral += (dt * self.y_error)
                if self.y_integral * ki_y  > 0.04:
                    self.y_integral = 0.04 / ki_y
                
                msg = Twist()
                msg.linear.y = self.y_error * kp_y + self.y_integral * ki_y
                #print(f'y упр: {self.y_error * kp + self.y_integral * ki}')
                self.publisher_.publish(msg)
            else:
                if not self.y_done:
                    msg = Twist()
                    print("done y")
                    self.publisher_.publish(msg)
                    self.y_done = True
                    self.y_integral = 0
            self.prev_time = self.get_clock().now()

            #print(self.choose_algoritm) 
            if abs(self.x_error) > 3:
                self.x_integral += (dt * self.x_error)
                if self.x_integral * ki_x  > 0.05:
                    self.x_integral = 0.05 / ki_x
                
                msg = Twist()
                msg.linear.x = self.x_error * kp_x + self.x_integral * ki_x
                print(f'x упр: {self.x_error * kp_y + self.x_integral * ki_x}')
                self.publisher_.publish(msg)
            else:
                if not self.x_done:
                    msg = Twist()
                    self.publisher_.publish(msg)
                    self.x_done = True
                    print("done x")
                    self.x_integral = 0

    

    # угловая ошибка
    def angle_control(self):
        print(self.choose_algoritm)
        if(self.choose_algoritm == "left" or self.choose_algoritm == "only_left"):
            self.angle_error = self.angle_error_left
        elif(self.choose_algoritm == "right" or self.choose_algoritm == "only_right"):
            self.angle_error = self.angle_error_right
        elif(self.choose_algoritm == "both"):
            self.angle_error = (self.angle_error_left + self.angle_error_right)/2
        # коэффицифенты регулятора
        ki = 0.0005
        kp = 0.007
        dt = self.get_clock().now().nanoseconds - self.prev_time.nanoseconds
        dt = dt * (10**-9)

        #Ограничение по времени 10с
        if ((self.get_clock().now().nanoseconds*(10**-9)- self.stop_time.nanoseconds*(10**-9)) > 10):
                msg = Twist()
                print("time is up")
                self.publisher_.publish(msg)
                self.integral = 0
                self.w_done = True
                self.choose_algoritm = "stop"

        if abs(self.angle_error) > 1:         

            self.stop_time = self.get_clock().now()
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

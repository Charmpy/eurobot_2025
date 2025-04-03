import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import math

class BoardDetector(Node):
    def __init__(self):
        super().__init__('board_detector')
        self.bridge = CvBridge()
        # подключение к топику камеры
        self.depth_sub = self.create_subscription(Image, '/robot_camera/depth_image', self.depth_callback, 10)
        # топик с изображением для отладки
        self.image_pub = self.create_publisher(Image, 'image_topic_2', 10)

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
        # ret,thresh4 = cv.threshold(depth_clipped,10,255,cv.THRESH_TOZERO)

        # ищем контуры
        edges = cv.Canny(depth_clipped, 20,20)


        # для отладки
        cdst = cv.cvtColor(depth_clipped, cv.COLOR_GRAY2BGR)
        
        # ищем линии
        lines = cv.HoughLines(edges, 1, np.pi / 180, 55, None, 0, 0)
        # отрисовываем
        if lines is not None:
            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                #  убираем что не интерсено
                if math.degrees(theta) < 80:
                    a = math.cos(theta)
                    b = math.sin(theta)
                    print(math.degrees(theta))
                    x0 = a * rho
                    y0 = b * rho
                    pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                    pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                    cv.line(cdst, pt1, pt2, (0,0,255), 1, cv.LINE_AA)
            
        print("---")
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(cdst))


def main(args=None):
    rclpy.init(args=args)
    node = BoardDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

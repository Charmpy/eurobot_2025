#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Инициализация CvBridge для преобразования изображений
        self.bridge = CvBridge()

        # Подписка на топик с изображениями
        self.subscription = self.create_subscription(
            Image, '/camera/image', self.image_callback, 10)
        self.subscription  # Предотвращаем предупреждение о неиспользовании

        # Публикация координат
        self.publisher_ = self.create_publisher(Float64MultiArray, '/aruco_coordinates', 10)

        # Параметры ArUco
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters_create()
        # Настройка параметров для улучшения обнаружения
        self.aruco_params.adaptiveThreshWinSizeMin = 3      # Минимальный размер окна
        self.aruco_params.adaptiveThreshWinSizeMax = 15     # Максимальный размер окна
        self.aruco_params.adaptiveThreshWinSizeStep = 1     # Шаг изменения размера
        self.aruco_params.adaptiveThreshConstant = 5        # Константа для порога
        self.aruco_params.minMarkerPerimeterRate = 0.01     # Минимальный периметр
        self.aruco_params.maxMarkerPerimeterRate = 2.0      # Максимальный периметр
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX  # Уточнение углов
        self.aruco_params.cornerRefinementWinSize = 5       # Размер окна уточнения
        self.aruco_params.minMarkerDistanceRate = 0.1     # Минимальное расстояние между маркерами

    def image_callback(self, msg):
        # Преобразование ROS Image в OpenCV формат
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Обнаружение ArUco-маркеров
        corners, ids, rejected = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            coordinates = Float64MultiArray()
            for i in range(len(ids)):
                # Вычисляем центр маркера как среднее значение углов
                corner = corners[i][0]
                center_x = np.mean(corner[:, 0])
                center_y = np.mean(corner[:, 1])
                # Добавляем координаты в массив
                coordinates.data.extend([float(center_x), float(center_y)])

            # Публикуем координаты
            self.publisher_.publish(coordinates)
            self.get_logger().info(f'Опубликовано {len(ids)} ArUco-маркеров: {coordinates.data}')

        
        

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
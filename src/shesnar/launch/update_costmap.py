#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class CostmapUpdater(Node):
    def __init__(self):
        super().__init__('costmap_updater')
        self.publisher_ = self.create_publisher(OccupancyGrid, '/local_costmap/costmap', 10)
        self.subscriber_ = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.costmap_callback, 10
        )
        self.timer = self.create_timer(0.01, self.update_costmap)  # Публикация каждую секунду (1 Гц)
        self.current_costmap = None  # Для хранения текущей карты

    def costmap_callback(self, msg):
        # Сохраняем текущую карту
        self.current_costmap = msg

    def update_costmap(self):
        if self.current_costmap is None:
            self.get_logger().info('Ожидаю данные costmap...')
            return

        # Копируем текущую карту
        msg = OccupancyGrid()
        msg.header.frame_id = 'odom'  # Оставляем base_link, так как это работает
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info = self.current_costmap.info
        msg.data = list(self.current_costmap.data)  # Копируем текущие данные

       
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        width = msg.info.width
        height = msg.info.height

        # Добавляем препятствие: прямоугольник от (x=0.5, y=0.5) до (x=1.0, y=1.0)
        for x in np.arange(0.5, 2.0, resolution):  # По X
            for y in np.arange(0.5, 2.0, resolution):  # По Y
                # Переводим координаты в индексы
                x_idx = int((x - origin_x) / resolution)
                y_idx = int((y - origin_y) / resolution)
                if 0 <= x_idx < width and 0 <= y_idx < height:
                    idx = y_idx * width + x_idx
                    msg.data[idx] = 100  # Устанавливаем значение "занято"

        self.publisher_.publish(msg)
        self.get_logger().info('Опубликовал обновлённый costmap с препятствием')

def main(args=None):
    rclpy.init(args=args)
    node = CostmapUpdater()
    rclpy.spin(node)  # Постоянный цикл публикации
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
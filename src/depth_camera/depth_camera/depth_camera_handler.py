import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R

import numpy as np
from sensor_msgs_py import point_cloud2 as pc2
import time

class PointCloudToLaserScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_laserscan')
        self.subscription = self.create_subscription(
            PointCloud2,
            'robot_camera/depth_image_points',  # Измени на свой топик
            self.pointcloud_callback,
            1
        )
        self.scan_publisher = self.create_publisher(LaserScan, '/robot_camera/scan_layer', 1)

        self.camera_tilt_angle = -30.0  # Наклон камеры в градусах
        self.rotation_matrix = R.from_euler('x', -self.camera_tilt_angle, degrees=True).as_matrix()


    def pointcloud_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        #print(points)

        # Define height layers (e.g., ground, mid, top)
        height_layer = -0.05  # Modify based on your needs
        scan_layer = []


        #for x, y, z in points:
        #    if height_layer - 0.02 <= z <= height_layer + 0.02:  # Allow small tolerance
        #        scan_layer.append((x, y))
        
        # выпонляется очень догло. Необходим более эффективное преобразование
        #print('points.shape is', points.shape, type(points[0][2]))
        scan_layer = [(x, y) for x, y, z in points if height_layer - 0.01 <= z <= height_layer + 0.01]
        #scan_layer = points[(points[:, 2] >= height_layer - 0.01) & (points[:, 2] <= height_layer + 0.01)]
        #print('speed.masure')
        angle_of_best_fit_line(scan_layer)
        
        self.publish_scan(scan_layer, 0)
        #time.sleep(0.5)



    def publish_scan(self, points, height):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = -np.pi
        scan_msg.angle_max = np.pi
        scan_msg.angle_increment = np.pi / 1800.0  # 1° шаг
        scan_msg.range_min = 0.1
        scan_msg.range_max = 1.0
        scan_msg.ranges = [float('inf')] * int((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment)

        for x, y in points:
            angle = np.arctan2(y, x)
            distance = np.hypot(x, y)
            index = int((angle - scan_msg.angle_min) / scan_msg.angle_increment)

            if 0 <= index < len(scan_msg.ranges):
                scan_msg.ranges[index] = min(scan_msg.ranges[index], distance)

        self.scan_publisher.publish(scan_msg)


def angle_of_best_fit_line(points):
    """
    Принимает список кортежей (x, y), аппроксимирует точки до прямой
    и возвращает угол наклона этой прямой в градусах.
    """
    # Разбираем входные данные
    x = np.array([p[0] for p in points])
    y = np.array([p[1] for p in points])
    
    # Выполняем линейную регрессию (метод наименьших квадратов)
    A = np.vstack([x, np.ones(len(x))]).T
    m, _ = np.linalg.lstsq(A, y, rcond=None)[0]  # m - наклон прямой
    
    # Вычисляем угол в градусах
    angle = np.degrees(np.arctan(m))
    print('ange in degrees is', angle)

    min_val = min(x**2 + y**2 for x, y in points)
    print('distance is', min_val)
    return angle



def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

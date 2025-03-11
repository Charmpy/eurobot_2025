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
            10
        )
        self.scan_publisher = self.create_publisher(LaserScan, '/robot_camera/scan_layer', 10)

        self.camera_tilt_angle = -30.0  # Наклон камеры в градусах
        self.rotation_matrix = R.from_euler('x', -self.camera_tilt_angle, degrees=True).as_matrix()

    """ 
    def pointcloud_callback(self, msg):
        # Читаем точки (x, y, z) и преобразуем в numpy-массив правильного формата
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        # Приведение данных к float32 (избавляемся от структурированного массива)
        points = np.vstack((points['x'], points['y'], points['z'])).T.astype(np.float32)

        # Проверяем форму массива (N, 3)
        if points.shape[1] != 3:
            self.get_logger().error(f"Unexpected point shape: {points.shape}")
            return

        # Поворот точек на +30° (чтобы компенсировать наклон камеры)
        rotated_points = np.dot(self.rotation_matrix, points.T).T

        # Фильтруем по высоте
        height_layers = [0.0, 0.5, 1.0]
        scan_layers = {h: [] for h in height_layers}

        for x, y, z in rotated_points:
            for h in height_layers:
                if h - 0.05 <= z <= h + 0.05:
                    scan_layers[h].append((x, y))

        for h, layer_points in scan_layers.items():
            self.publish_scan(layer_points, h)
            time.sleep(1)
    """


    def pointcloud_callback(self, msg):
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        #print(points)

        # Define height layers (e.g., ground, mid, top)
        height_layers = [-0.1]  # Modify based on your needs
        scan_layers = {h: [] for h in height_layers}


        for x, y, z in points:
            for h in height_layers:
                if h - 0.02 <= z <= h + 0.02:  # Allow small tolerance
                    scan_layers[h].append((x, y))
        
        for h, layer_points in scan_layers.items():
            self.publish_scan(layer_points, h)
            #time.sleep(0.5)



    def publish_scan(self, points, height):
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'base_link'
        scan_msg.angle_min = -np.pi
        scan_msg.angle_max = np.pi
        scan_msg.angle_increment = np.pi / 180.0  # 1° шаг
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

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToLaserScan()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

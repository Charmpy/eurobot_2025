###
### Before start Node:
### export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libunwind.so.8
###
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class CusCamera(Node):
    def __init__(self):
        super().__init__('cus_camera')
        self.get_logger().info(f"OpenCV version: {cv2.__version__}")

        self.odom_pub = self.create_publisher(Odometry, 'camera_odom', 1000)

        qos_profile = QoSProfile(
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.callback,
            qos_profile=qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        
        self.K = np.array([[ 1.05477276e+03, -1.10039944e-01,  1.07993991e+03],
                        [ 0.00000000e+00,  1.05437012e+03,  7.33474904e+02],
                        [ 0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
        self.D = np.array([[-0.03337077], [-0.00093301], [-0.00130544], [ 0.0011649 ]])
        
        
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        # Aruco positions in playground coordinates in format:
        # <Aruco ID>: <corners in playground coordinates: top left, top right, bottom right, bottom left> 
        Aruco = {
            20: np.array([[550, 550, 0], [650, 550, 0], [650, 650, 0], [550, 650, 0]], dtype=np.float32),
            21: np.array([[2350, 550, 0], [2450, 550, 0], [2450, 650, 0], [2350, 650, 0]], dtype=np.float32),
            22: np.array([[550, 1350, 0], [650, 1350, 0], [650, 1450, 0], [550, 1450, 0]], dtype=np.float32),
            23: np.array([[2350, 1350, 0], [2450, 1350, 0], [2450, 1450, 0], [2350, 1450, 0]], dtype=np.float32)
        }
        objPoints = np.fromiter(Aruco.values(), dtype=object)
        ids = np.fromiter(Aruco.keys(), dtype=float)
        # print(objPoints, ids)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.Board(objPoints, dictionary, ids)

        self.marker_length = 80
        self.object_corners = np.array([
            [[-self.marker_length//2,  self.marker_length//2, 0]],  # Левый верхний
            [[ self.marker_length//2,  self.marker_length//2, 0]],  # Правый верхний
            [[ self.marker_length//2, -self.marker_length//2, 0]],  # Правый нижний
            [[-self.marker_length//2, -self.marker_length//2, 0]]   # Левый нижний
        ], dtype=np.float32)

        self.camera_matrix = None
        self.is_calibrated = False
        self.start_time = 0
        self.end_time = 0
        self.get_logger().info('Node started')


    def transform(self, coordinates, z=0):
        # self.get_logger().info(f"Alpha: \n{alpha}")
        center = np.mean(coordinates, axis=0).astype(int)
        center = np.array([center[0], center[1], 1])
        
        alpha = (-z + self.r[-1,:] @ self.t)/(self.r[-1,:] @ center)

        pose = self.r @ (alpha * center - self.t)
        pose[1] = 2000 - pose[1]
        pose /= 1000

        coordinates = np.array(coordinates, dtype=np.float32)
        coordinates = coordinates.reshape(4, 1, 2)
        # print(object_corners.shape, corner_coord.shape)

        retval, rvec, tvec = cv2.solvePnP(self.object_corners, coordinates, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        # cv2.drawFrameAxes(
        #     self.image, 
        #     self.camera_matrix, 
        #     self.dist_coeffs, 
        #     rvec, 
        #     tvec, 
        #     self.marker_length/2
        # )
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        theta = np.arctan2(rotation_matrix[1,0], rotation_matrix[0,0])

        return pose[:2], theta

    def find_markers(self, calibrate=False):
        marker_corners, marker_IDs, reject = self.detector.detectMarkers(self.image)
        # self.get_logger().info(f"Founded markres with ids: {marker_IDs.ravel()}")
        
        if marker_corners:
            if calibrate:
                return marker_corners, marker_IDs
            else:
                result = {}
                for ids, corners in zip(marker_IDs, marker_corners):
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    # top_left = corners[0].ravel()
                    # top_right = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    # bottom_left = corners[3].ravel()
                    # center_coordinates = (int(top_left[0] + top_right[0]) // 2, int(top_left[1] + top_right[1]) // 2)
                    # self.image = cv2.circle(self.image, center_coordinates, 5, (0, 255, 0), 4)
                    cv2.putText(self.image, str(ids),
                            (bottom_right[0], bottom_right[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                    # center = (int(top_left[0] + bottom_right[0]) // 2, int(top_left[1] + bottom_right[1]) // 2, 1)
                    # center = np.array(center)
                    
                    result[ids[0]] = corners
                    # print(result[ids[0]])
                    # result[ids[0]] = center, np.hstack((top_left, 1))
                return result
        else:
            return None
    
    def calibrate(self):
        marker_corners, marker_IDs = self.find_markers(True)
        objPoints, imgPoints = self.board.matchImagePoints(marker_corners, marker_IDs)
        print(objPoints)
        print(imgPoints)
        retval, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
        # self.get_logger().info(f"Tvec: \n{tvec}")
        # self.get_logger().info(f"Rvec: {rvec}")
        # self.get_logger().info(f"Rmat: \n{cv2.Rodrigues(rvec)[0]}")

        # self.image = cv2.drawFrameAxes(self.image, self.camera_matrix, self.dist_coeffs, rvec, tvec, length=1000)

        r_mat = cv2.Rodrigues(rvec)[0]
        Rt = np.hstack((r_mat, tvec))
        T = self.camera_matrix @ Rt
        
        self.r = np.linalg.inv(T[:,:-1])  # final rotate matrix
        self.t = T[:,-1]  # final translate vector

        self.get_logger().info(f"Calibrated succesfuly!")
        self.get_logger().info(f"r: \n{self.r}")
        self.get_logger().info(f"t: {self.t}")

        self.set_default()
        self.is_calibrated = True


    def set_default(self):
        markers = self.find_markers()
        pose, theta = self.transform(markers[69], 435)
        # self.default_theta = 3.874187464676028
        self.default_theta = theta
        self.get_logger().info(f"Default theta: {self.default_theta}")
        self.last_x, self.last_y, self.last_theta = pose[1], -pose[0], 0
        self.get_logger().info(f"Start odometry: {self.last_x, self.last_y, self.last_theta}")
        self.last_time = self.get_clock().now().nanoseconds


    def send_tf(self, x, y, theta):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = y
        t.transform.translation.y = -x
        t.transform.translation.z = 0.01

        q = quaternion_from_euler(0, 0, -theta)
        # self.get_logger().info(f"Orientation: {q}")
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # self.tf_broadcaster.sendTransform(t)

    def send_odometry(self, x, y, theta):
        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map'
        
        # self.get_logger().info(f"Time: {self.get_clock().now().nanoseconds}")

        
        # dt = (self.current_time - self.last_time) / 10**9
        dt = 0.1
        # self.get_logger().info(f"Delta time: {dt}")

        # self.get_logger().info(f"Pose: {y, -x}, theta: {theta}")

        q = quaternion_from_euler(0, 0, -theta)
        # set the position
        odom.pose.pose.position.x = y
        odom.pose.pose.position.y = -x
        odom.pose.pose.position.z = 0.01;        
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        vel_x, vel_y, vel_w = (y - self.last_x)/dt, (-x - self.last_y)/dt, (-theta - self.last_theta)/dt
        # self.get_logger().info(f"Velosity: {vel_x, vel_y, vel_w}")

        # set the velocity
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.linear.y = vel_y
        # odom.twist.twist.linear.x = 0.00
        # odom.twist.twist.linear.y = 0.00
        odom.twist.twist.linear.z = 0.00

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        # odom.twist.twist.angular.z = 0.0
        odom.twist.twist.angular.z = vel_w

        self.odom_pub.publish(odom)

        self.end_time = self.get_clock().now().nanoseconds
        
        self.last_x = y
        self.last_y = -x
        self.last_theta = -theta

    def callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            self.get_logger().warning("No image")
        self.image = cv2.flip(self.image, -1)
        self.start_time = self.get_clock().now().nanoseconds
        # np_arr = np.frombuffer(msg.data, np.uint8)
        # self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        h,  w = self.image.shape[:2]
        if self.camera_matrix is None:
            self.camera_matrix = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                self.K, self.D, (w, h), np.eye(3), balance=0 # balance=0 (обрезка краёв) ... 1 (сохранение всех пикселей)
            )
            self.dist_coeffs = np. array([0, 0, 0, 0, 0])

        undistorted = cv2.fisheye.undistortImage(self.image, self.K, self.D, None, self.camera_matrix)

        self.image = undistorted
        
        if self.is_calibrated:
            try:
                markers = self.find_markers()
                # self.get_logger().info(f"Founded markres: {markers.keys()}")
                pose, theta = self.transform(markers[69], 435)
                # self.send_tf(pose[0], pose[1], theta)
                self.send_odometry(pose[0], pose[1], theta)
                self.get_logger().info(f"Odometry: {pose[0], pose[1], np.degrees(theta)}")
            except Exception as e:
                self.get_logger().warning(f"Error finding: {e}")
        else:
            try:
                self.calibrate()

            # # self.get_logger().info(f"Coordinates: {result}")
            except Exception as e:
                self.get_logger().error(f'Error calibrating: {e}')
        # self.get_logger().info(self.get_clock().now().to_msg())
        self.end_time = self.get_clock().now().nanoseconds
        # print(self.start_time, self.end_time)
        
        
        cv2.putText(self.image, "Ping = " + str((self.end_time - self.start_time)//(10**6)) + "ms", (self.image.shape[1] - 400, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow("Camera Image", self.image)
        key = cv2.waitKey(1)
        if key == ord('c'):
            pass
        if key == ord('q'):
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = CusCamera()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
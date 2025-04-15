import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ImageSubscriber(Node):
    def __init__(self):
        
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image', 
            self.image_callback,
            10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.bridge = CvBridge()
        

        self.get_logger().info(f"OpenCV version: {cv2.__version__}")
        self.get_logger().info('Image subscriber started')

        self.camera_matrix = np.array([[761.80910110473633, 0., 960], [0., 761.80913686752319, 540], [0., 0., 1.]], dtype=np.float32)
        self.dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)

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
        
        self.is_calibrated = False

    def transform(self, coord, z=0):
        alpha = (-z + self.r[-1,:] @ self.t)/(self.r[-1,:] @ coord)
        # self.get_logger().info(f"Alpha: \n{alpha}")
        pose = self.r @ (alpha * coord - self.t)
        pose[1] = 2000 - pose[1]
        return pose[:2]

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
                    top_left = corners[0].ravel()
                    # top_right = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    # bottom_left = corners[3].ravel()
                    
                    cv2.putText(self.image, str(ids),
                            (bottom_right[0], bottom_right[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                    center = (int(top_left[0] + bottom_right[0]) // 2, int(top_left[1] + bottom_right[1]) // 2, 1)
                    center = np.array(center)
                    result[ids[0]] = center
                return result
        else:
            return None
    
    def calibrate(self):
        marker_corners, marker_IDs = self.find_markers(True)
        objPoints, imgPoints = self.board.matchImagePoints(marker_corners, marker_IDs)
        retval, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
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

        self.is_calibrated = True

    def send_tf(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        cv2.imshow("Camera Image", self.image)
        key = cv2.waitKey(1)
        if key == ord('c'):
            pass
        if key == ord('q'):
            raise SystemExit


        if self.is_calibrated:
            try:
                markers = self.find_markers()
                # self.get_logger().info(f"Founded markres: {markers.keys()}")
                pose = self.transform(markers[69], 435)
                self.get_logger().info(f"Pose: \n{pose}")
            except Exception as e:
                self.get_logger().warning(f"Error finding: {e}")
        else:
            try:
                self.calibrate()

            # # self.get_logger().info(f"Coordinates: {result}")
            except Exception as e:
                self.get_logger().error(f'Error calibrating: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
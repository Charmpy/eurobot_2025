import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        print(cv2.__version__)
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image', 
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Image subscriber started')
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
        print(objPoints, ids)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.board = cv2.aruco.Board(objPoints, dictionary, ids)

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imshow("Camera Image", image)
            marker_corners, marker_IDs, reject = self.detector.detectMarkers(image)
            self.get_logger().info(f"Founded markres with ids: {marker_IDs.ravel()}")
            result = {}
            if marker_corners:
                for ids, corners in zip(marker_IDs, marker_corners):
                    corners = corners.reshape(4, 2)
                    corners = corners.astype(int)
                    top_left = corners[0].ravel()
                    # top_right = corners[1].ravel()
                    bottom_right = corners[2].ravel()
                    # bottom_left = corners[3].ravel()
                    
                    cv2.putText(image, str(ids),
                            (bottom_right[0], bottom_right[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                    center = (int(top_left[0] + bottom_right[0]) // 2, int(top_left[1] + bottom_right[1]) // 2)
                    center = np.array(center)
                    result[ids[0]] = center

            camera_matrix = np.array([[761.80910110473633, 0., 960], [0., 761.80913686752319, 540], [0., 0., 1.]], dtype=np.float32)
            dist_coeffs = np.array([0, 0, 0, 0, 0], dtype=np.float32)
            objPoints, imgPoints = self.board.matchImagePoints(marker_corners, marker_IDs)
            retval, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, camera_matrix, dist_coeffs)
            self.get_logger().info(f"Rvec, tvec: {rvec, tvec}")
            image = cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, length=1000 )
            self.get_logger().info(f"Coordinates: {result}")
            cv2.imshow("Camera Image", image)
            key = cv2.waitKey(1)
            if key == ord('q'):
                raise SystemExit

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


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
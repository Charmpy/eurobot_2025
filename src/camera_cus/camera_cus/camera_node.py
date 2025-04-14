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
        # self.get_logger().info(f"Matrix: \n{alpha}")
        pose = self.r @ (alpha * coord - self.t)
        pose[1] = 2000 - pose[1]
        return pose[:2]

    def find_markers(self):
        marker_corners, marker_IDs, reject = self.detector.detectMarkers(self.image)
        # self.get_logger().info(f"Founded markres with ids: {marker_IDs.ravel()}")
        
        if marker_corners:
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
            return result, marker_corners, marker_IDs
        else:
            return None
    
    def calibrate(self):
        pass

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # cv2.imshow("Camera Image", image)
            markers, marker_corners, marker_IDs = self.find_markers()
            self.get_logger().info(f"Founded markres: {markers.keys()}")
            
            objPoints, imgPoints = self.board.matchImagePoints(marker_corners, marker_IDs)
            retval, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, self.camera_matrix, self.dist_coeffs)
            # self.get_logger().info(f"Tvec: \n{tvec}")
            # self.get_logger().info(f"Rvec: {rvec}")
            # self.get_logger().info(f"Rmat: \n{cv2.Rodrigues(rvec)[0]}")
            r_mat = cv2.Rodrigues(rvec)[0]
            Rt = np.hstack((r_mat, tvec))
            T = self.camera_matrix @ Rt
            
            self.r = np.linalg.inv(T[:,:-1])  # final rotate matrix
            self.t = T[:,-1]  # final translate vector

            try:
                pose = self.transform(markers[69], 435)
                self.get_logger().info(f"Pose: \n{pose}")
            except:
                self.get_logger().warning(f"Marker not finded: {69}")
            # # self.get_logger().info(f"Matrix: \n{np.array(Rt, dtype=np.float16)}")
            # # self.get_logger().info(f"Matrix: \n{Transform @ Transform_inv}")
            # # self.get_logger().info(f"Matrix: \n{Transform_inv @ [600, 600, 0, 1]}")

            # image = cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, length=1000)
            # # self.get_logger().info(f"Coordinates: {result}")
            # # cv2.imshow("Camera Image", image)
            # # key = cv2.waitKey(1)
            # # if key == ord('c'):
            # #     pass
            # # if key == ord('q'):
            # #     raise SystemExit

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
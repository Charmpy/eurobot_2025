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

    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # !!!!
            image = cv2.flip(image, 0)   # DELETE THIS AFTER FIXING PLAYMAT
            # !!!!
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
                            (top_left[0], top_left[1] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)

                    center = (int(top_left[0] + bottom_right[0]) // 2, int(top_left[1] + bottom_right[1]) // 2)
                    center = np.array(center)
                    result[ids[0]] = center
            cv2.imshow("Camera Image", image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import os
import cv2
import numpy as np
from datetime import datetime
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError

class CompressedImage2Dataset(Node):
    def __init__(self):
        super().__init__('compressedImage2dataset')

        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.subscription = self.create_subscription(
            CompressedImage,
            '/Head/D435_H/color/image_raw/compressed',
            self.cb_image_input,
            qos_profile
        )
        self.count_input = 0
        self.count_output = 0
        self.bridge = CvBridge()
        self.image_folder = 'images'
        self.date = datetime.now().strftime("%Y%m%d%H%M%S")
        if not os.path.exists(os.path.join(self.image_folder, self.date)):
            os.makedirs(os.path.join(self.image_folder, self.date))
    
    def cb_image_input(self, msg):
        image_np = self.compressed_imgmsg_to_cv2(msg)
        if self.count_input < 10:
            image_name = f'input_0000{self.count_input}.png'
        elif self.count_input < 100:
            image_name = f'input_000{self.count_input}.png'
        elif self.count_input < 1000:
            image_name = f'input_00{self.count_input}.png'
        elif self.count_input < 10000:
            image_name = f'input_0{self.count_input}.png'
        else:
            image_name = f'input_{self.count_input}.png'
        image_path = os.path.join(self.image_folder, self.date, image_name)
        cv2.imwrite(image_path, image_np)

        self.count_input += 1

    def compressed_imgmsg_to_cv2(
            self, img_msg: CompressedImage, encoding="passthrough"
        ) -> np.ndarray:
            """CompressedImageメッセージをcv2形式の画像に変換する

            Args:
                img_msg (CompressedImage): CompressedImageメッセージ
                encoding (str, optional): エンコーディング. Defaults to "passthrough".

            Returns:
                np.ndarray: cv2形式の画像．
            """
            try:
                cv_img = self.bridge.compressed_imgmsg_to_cv2(img_msg, encoding)
                return cv_img
            except CvBridgeError as e:
                self.get_logger().warn(f"{e}")
                self.get_logger().warn(f"CvBridge FAILURE.")
                return None


def main(args=None):
    rclpy.init(args=args)

    ci2d = CompressedImage2Dataset()
    rclpy.spin(ci2d)

    ci2d.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
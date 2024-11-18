import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2
import numpy as np

class ImageToVideoNode(Node):
    def __init__(self):
        super().__init__('image_to_video_node')
        qos_profile = rclpy.qos.qos_profile_sensor_data
        self.subscription = self.create_subscription(
            CompressedImage, 
            '/Base_front/D435if/color/image_raw/compressed', 
            self.image_callback, 
            qos_profile
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.frames = []
        self.output_filename = 'base_video.mp4'
        self.video_writer = None

    def image_callback(self, msg):
        cv_image = self.compressed_imgmsg_to_cv2(msg)
        self.frames.append(cv_image)

    def save_video(self):
        if not self.frames:
            return

        first_frame = self.frames[0]
        height, width, _ = first_frame.shape
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 'mp4v' is a common codec for MP4
        self.video_writer = cv2.VideoWriter(self.output_filename, fourcc, 10, (width, height))

        for frame in self.frames:
            self.video_writer.write(frame)

        if self.video_writer is not None:
            self.video_writer.release()

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
        super().destroy_node()

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
    image_to_video_node = ImageToVideoNode()
    try:
        rclpy.spin(image_to_video_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_to_video_node.save_video()
        image_to_video_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
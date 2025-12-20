#!/home/karol/ws/HITNET_venv/bin/python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from message_filters import Subscriber, ApproximateTimeSynchronizer

from hitnet import HitNet, ModelType, draw_disparity


class HitnetStereoNode(Node):
    def __init__(self):
        super().__init__("hitnet_stereo_node")

        # Parametry (łatwo zmienisz z linii poleceń)
        self.declare_parameter("left_topic", "/camera_front/left/image_raw")
        self.declare_parameter("right_topic", "/camera_front/right/image_raw")
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("slop", 0.05)
        self.declare_parameter("model_path", "models/eth3d.pb")

        left_topic = self.get_parameter("left_topic").get_parameter_value().string_value
        right_topic = self.get_parameter("right_topic").get_parameter_value().string_value
        queue_size = self.get_parameter("queue_size").get_parameter_value().integer_value
        slop = self.get_parameter("slop").get_parameter_value().double_value
        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        # HitNet
        model_type = ModelType.eth3d
        self.hitnet_depth = HitNet(model_path, model_type)

        # CV Bridge
        self.bridge = CvBridge()

        # message_filters subscribers dla ROS2:
        # UWAGA: w ROS2 przekazujesz node jako 1. argument do Subscriber
        self.left_sub = Subscriber(self, Image, left_topic)
        self.right_sub = Subscriber(self, Image, right_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=queue_size,
            slop=slop
        )
        self.sync.registerCallback(self.stereo_callback)

        self.get_logger().info("HitNet stereo node started (ROS2)")

    def stereo_callback(self, left_msg: Image, right_msg: Image):
        try:
            # ROS2 Image -> OpenCV (BGR)
            left_img = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right_img = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")

            # HitNet zwykle oczekuje RGB
            left_rgb = cv2.cvtColor(left_img, cv2.COLOR_BGR2RGB)
            right_rgb = cv2.cvtColor(right_img, cv2.COLOR_BGR2RGB)

            disparity_map = self.hitnet_depth(left_rgb, right_rgb)
            color_disparity = draw_disparity(disparity_map)

            combined = np.hstack((left_rgb, right_rgb, color_disparity))

            cv2.imshow("Estimated disparity", combined)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"stereo_callback error: {e}")


def main():
    rclpy.init()
    node = HitnetStereoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
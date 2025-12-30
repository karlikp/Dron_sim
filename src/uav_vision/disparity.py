#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from hitnet import HitNet, ModelType, draw_disparity


# FX_PX = 500.0        # [px] – dostosuj do kamery w Gazebo
# BASELINE_M = 0.20    # [m]  – dystans między kamerami
# MIN_DISPARITY = 0.1  # [px] – zabezpieczenie przed dzieleniem przez zero

@dataclass(frozen=True)
class StereoParams:
    fx_px: float
    baseline_m: float


class StereoDepthCalculator:
    """Converts disparity [px] to depth [m] using Z = fx * B / d."""

    def __init__(self, min_disparity_px: float = 0.1) -> None:
        self._min_disparity_px = float(min_disparity_px)

    def depth_from_disparity(self, disparity_px: np.ndarray, stereo: StereoParams) -> np.ndarray:
        if disparity_px.ndim != 2:
            raise ValueError(f"Expected 2D disparity map, got shape={disparity_px.shape}")

        disp = disparity_px.astype(np.float32)
        invalid = disp < self._min_disparity_px #tablica NumPy typu bool

        depth_m = (stereo.fx_px * stereo.baseline_m) / np.maximum(disp, self._min_disparity_px)
        depth_m[invalid] = np.nan  # bardziej czytelne niż 0, łatwo maskować w wizualizacji
        return depth_m


class HitnetStereoNode(Node):
    def __init__(self) -> None:
        super().__init__("hitnet_stereo_node")

        # Topics
        self.declare_parameter("left_topic", "/camera_front/left/image_raw")
        self.declare_parameter("right_topic", "/camera_front/right/image_raw")
        self.declare_parameter("left_info_topic", "/camera_front/left/camera_info")
        self.declare_parameter("right_info_topic", "/camera_front/right/camera_info")

        # Sync
        self.declare_parameter("queue_size", 5)
        self.declare_parameter("slop", 0.05)

        # HitNet
        self.declare_parameter("model_path", "models/middlebury.pb")
        self.declare_parameter("model_type", "middlebury")  # eth3d / middlebury / kitti (jeśli wspierane w Twojej wersji)

        # Fallback stereo params (gdy brak CameraInfo)
        self.declare_parameter("fx_px", 500.0)
        self.declare_parameter("baseline_m", 0.20)

        # Depth visualization range
        self.declare_parameter("depth_min_m", 0.0)
        self.declare_parameter("depth_max_m", 20.0)

        # Depth publish (opcjonalnie)
        self.declare_parameter("publish_depth", True)
        self.declare_parameter("depth_left_topic", "/stereo/depth_left")
        self.declare_parameter("depth_right_topic", "/stereo/depth_right")

        self._bridge = CvBridge()
        self._depth_calc = StereoDepthCalculator(min_disparity_px=0.1)

        self._stereo_params: Optional[StereoParams] = None
        self._left_info: Optional[CameraInfo] = None
        self._right_info: Optional[CameraInfo] = None

        self._init_hitnet()
        self._init_camera_info_subscribers()
        self._init_image_sync()
        self._init_publishers()

        self.get_logger().info("HitNet stereo node started (disparity + depth)")

    # -------------------------- init helpers --------------------------

    def _init_hitnet(self) -> None:
        model_path = self.get_parameter("model_path").get_parameter_value().string_value
        model_type_str = self.get_parameter("model_type").get_parameter_value().string_value.lower()

        model_type_map = {
            "eth3d": ModelType.eth3d,
            "middlebury": ModelType.middlebury,
#            "kitti": ModelType.kitti,
        }
        if model_type_str not in model_type_map:
            raise ValueError(f"Unsupported model_type='{model_type_str}'. Choose one of {list(model_type_map.keys())}")

        self._hitnet = HitNet(model_path, model_type_map[model_type_str])

    def _init_camera_info_subscribers(self) -> None:
        left_info_topic = self.get_parameter("left_info_topic").get_parameter_value().string_value
        right_info_topic = self.get_parameter("right_info_topic").get_parameter_value().string_value

        self._left_info_sub = self.create_subscription(CameraInfo, left_info_topic, self._on_left_info, 10)
        self._right_info_sub = self.create_subscription(CameraInfo, right_info_topic, self._on_right_info, 10)

    def _init_image_sync(self) -> None:
        left_topic = self.get_parameter("left_topic").get_parameter_value().string_value
        right_topic = self.get_parameter("right_topic").get_parameter_value().string_value
        queue_size = self.get_parameter("queue_size").get_parameter_value().integer_value
        slop = self.get_parameter("slop").get_parameter_value().double_value

        self._left_sub = Subscriber(self, Image, left_topic)
        self._right_sub = Subscriber(self, Image, right_topic)

        self._sync = ApproximateTimeSynchronizer([self._left_sub, self._right_sub], queue_size, slop)
        self._sync.registerCallback(self._on_stereo_pair)

    def _init_publishers(self) -> None:
        publish_depth = self.get_parameter("publish_depth").get_parameter_value().bool_value
        if not publish_depth:
            self._depth_left_pub = None
            self._depth_right_pub = None
            return

        depth_left_topic = self.get_parameter("depth_left_topic").get_parameter_value().string_value
        depth_right_topic = self.get_parameter("depth_right_topic").get_parameter_value().string_value

        self._depth_left_pub = self.create_publisher(Image, depth_left_topic, 10)
        self._depth_right_pub = self.create_publisher(Image, depth_right_topic, 10)

    # -------------------------- CameraInfo parsing --------------------------

    def _on_left_info(self, msg: CameraInfo) -> None:
        self._left_info = msg
        self._try_update_stereo_params()

    def _on_right_info(self, msg: CameraInfo) -> None:
        self._right_info = msg
        self._try_update_stereo_params()

    def _try_update_stereo_params(self) -> None:
        # Jeśli mamy CameraInfo, pobieramy fx i baseline z macierzy projekcji P
        if self._left_info is None or self._right_info is None:
            return

        fx = float(self._left_info.k[0])  # K[0] = fx
        baseline = self._baseline_from_projection(self._right_info)

        if fx > 0.0 and baseline > 0.0:
            self._stereo_params = StereoParams(fx_px=fx, baseline_m=baseline)

    @staticmethod
    def _baseline_from_projection(right_info: CameraInfo) -> float:
        """
        W stereo po rektyfikacji baseline można wyliczyć z P:
        P = [fx 0 cx Tx;
             0 fy cy 0;
             0  0  1 0]
        gdzie Tx = -fx * B  =>  B = -Tx / fx
        """
        P = right_info.p  # length 12
        fx = float(P[0])
        Tx = float(P[3])
        if fx == 0.0:
            return 0.0
        baseline = -Tx / fx
        return float(abs(baseline))

    def _get_stereo_params(self) -> StereoParams:
        if self._stereo_params is not None:
            return self._stereo_params

        # fallback parametry (gdy brak CameraInfo)
        fx = self.get_parameter("fx_px").get_parameter_value().double_value
        baseline = self.get_parameter("baseline_m").get_parameter_value().double_value
        return StereoParams(fx_px=float(fx), baseline_m=float(baseline))

    # -------------------------- main callback --------------------------

    def _on_stereo_pair(self, left_msg: Image, right_msg: Image) -> None:
        try:
            left_bgr = self._bridge.imgmsg_to_cv2(left_msg, desired_encoding="bgr8")
            right_bgr = self._bridge.imgmsg_to_cv2(right_msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Failed to convert ROS Image to cv2: {exc}")
            return

        left_rgb = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2RGB)
        right_rgb = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2RGB)

        # Dysparycja (lewa) i (prawa) – przez odwrócenie wejść
        disp_left = self._hitnet(left_rgb, right_rgb).astype(np.float32)
        disp_right = self._hitnet(right_rgb, left_rgb).astype(np.float32)

        stereo = self._get_stereo_params()
        depth_left = self._depth_calc.depth_from_disparity(disp_left, stereo)
        depth_right = self._depth_calc.depth_from_disparity(disp_right, stereo)

        self._publish_depth(depth_left, left_msg.header, is_left=True)
        self._publish_depth(depth_right, right_msg.header, is_left=False)

        self._show_debug_views(disp_left, disp_right, depth_left, depth_right)

    # -------------------------- output helpers --------------------------

    def _publish_depth(self, depth_m: np.ndarray, header, is_left: bool) -> None:
        pub = self._depth_left_pub if is_left else self._depth_right_pub
        if pub is None:
            return

        # sensor_msgs/Image: encoding "32FC1" (m)
        depth_msg = self._bridge.cv2_to_imgmsg(depth_m.astype(np.float32), encoding="32FC1")
        depth_msg.header = header
        pub.publish(depth_msg)

    def _show_debug_views(
        self,
        disp_left: np.ndarray,
        disp_right: np.ndarray,
        depth_left: np.ndarray,
        depth_right: np.ndarray,
    ) -> None:
        disp_left_vis = self._vis_disparity(disp_left)
        disp_right_vis = self._vis_disparity(disp_right)

        depth_min = float(self.get_parameter("depth_min_m").get_parameter_value().double_value)
        depth_max = float(self.get_parameter("depth_max_m").get_parameter_value().double_value)

        depth_left_vis = self._vis_depth(depth_left, depth_min, depth_max)
        depth_right_vis = self._vis_depth(depth_right, depth_min, depth_max)

        
                         
        top = np.hstack([self._with_title(disp_left_vis, "dysparycja dla lewej"),
                         self._with_title(disp_right_vis, "dysparycja dla prawej")])
        bottom = np.hstack([self._with_title(depth_left_vis, "glebia dla lewej"),
                            self._with_title(depth_right_vis, "glebia dla prawej")])

        grid = np.vstack([top, bottom])
        cv2.imshow("Stereo: disparity + depth", grid)
        cv2.waitKey(1)

    @staticmethod
    def _vis_disparity(disparity_px: np.ndarray) -> np.ndarray:
        # draw_disparity zwykle robi sensowną normalizację/kolorowanie
        vis = draw_disparity(disparity_px)
        if vis.ndim == 2:
            vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
        # HitNet zwraca RGB? draw_disparity często zwraca BGR; dopasuj jeśli masz odwrócone kanały.
        return vis

    @staticmethod
    def _vis_depth(depth_m: np.ndarray, vmin: float, vmax: float) -> np.ndarray:
        depth = depth_m.copy()
        depth[np.isnan(depth)] = vmax  # nieważne piksele jako "daleko" dla czytelności

        depth = np.clip(depth, vmin, vmax)
        norm = (depth - vmin) / max(vmax - vmin, 1e-6)
        img_u8 = (norm * 255.0).astype(np.uint8)

        colored = cv2.applyColorMap(img_u8, cv2.COLORMAP_JET)
        return colored

    @staticmethod
    def _with_title(img_bgr: np.ndarray, title: str) -> np.ndarray:
        out = img_bgr.copy()
        cv2.putText(out, title, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2, cv2.LINE_AA)
        return out


def main() -> None:
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

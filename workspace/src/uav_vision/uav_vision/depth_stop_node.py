#!/usr/bin/env python3
from __future__ import annotations

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from uav_vision.depth_processing import (
    compute_min_depth,
    compute_roi,
    declare_depth_parameters,
    update_stop_state,
)
from uav_vision.measurement_map import MeasurementMap, declare_measurement_map_parameters
from uav_vision.motion_state import MotionState, declare_motion_parameters
from uav_vision.px4_reaction import Px4Reaction, declare_px4_reaction_parameters


class DepthStopNode(Node):
    """
    depth_stop_node:
      - Subscribes: depth_image (sensor_msgs/Image, expected 32FC1 in meters)
      - Optional gating: forward velocity and altitude
      - Publishes: std_msgs/Bool /obstacle_stop
      - Optional: publishes PX4 HOLD/offboard zero-velocity reaction
      - Optional: marks GPS points that require measurement on a local Mapbox view
    """

    def __init__(self) -> None:
        super().__init__("depth_stop_node")
        self._bridge = CvBridge()

        declare_depth_parameters(self)
        declare_motion_parameters(self)
        declare_px4_reaction_parameters(self)
        declare_measurement_map_parameters(self)

        self._stop_pub = self.create_publisher(Bool, self.get_parameter("stop_topic").value, 10)
        self._stop_state = False

        self._motion = MotionState(self, self._px4_qos)
        self._motion.init_subscriptions()

        self._px4_reaction = Px4Reaction(self, lambda: self._stop_state)
        self._measurement_map = MeasurementMap(self, self._px4_qos)

        self._log_min_depth = bool(self.get_parameter("log_min_depth").value)
        self._log_period_s = float(self.get_parameter("log_period_s").value)
        self._last_log_time = self.get_clock().now()

        depth_topic = self.get_parameter("depth_topic").value
        self._depth_sub = self.create_subscription(Image, depth_topic, self._on_depth, 10)

        self.get_logger().info(
            f"DepthStopNode started. depth_topic={depth_topic}, stop_topic={self.get_parameter('stop_topic').value}"
        )

    def _px4_qos(self) -> QoSProfile:
        return QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _on_depth(self, msg: Image) -> None:
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            self.get_logger().error(f"Depth conversion failed: {exc}")
            return

        if depth.ndim != 2:
            self.get_logger().error(f"Expected 2D depth image, got shape={depth.shape}")
            return

        roi = compute_roi(depth.shape, self)
        depth_roi = depth[roi.y0:roi.y1, roi.x0:roi.x1].astype(np.float32)
        stats = compute_min_depth(depth_roi, self)

        if self._is_altitude_gated(stats.min_depth, stats.valid_fraction):
            return

        if self._is_velocity_gated(stats.min_depth, stats.valid_fraction):
            return

        min_valid_fraction = float(self.get_parameter("min_valid_fraction").value)
        if stats.valid_fraction < min_valid_fraction:
            self._maybe_log(stats.min_depth, stats.valid_fraction, insufficient=True)
            self._publish_stop(self._stop_state)
            return

        self._stop_state = update_stop_state(self._stop_state, stats.min_depth, self)

        self._maybe_log(stats.min_depth, stats.valid_fraction)
        self._publish_stop(self._stop_state)

    def _is_altitude_gated(self, min_depth: float, valid_fraction: float) -> bool:
        if not bool(self.get_parameter("use_altitude_gating").value):
            return False

        if not self._motion.is_below_hold_altitude():
            return False

        self._stop_state = False
        self._measurement_map.reset_stop_edge()
        self._maybe_log(min_depth, valid_fraction, altitude_gated=True)
        self._publish_stop(False)
        return True

    def _is_velocity_gated(self, min_depth: float, valid_fraction: float) -> bool:
        if not bool(self.get_parameter("use_velocity_gating").value):
            return False

        if self._motion.is_moving_forward():
            return False

        self._maybe_log(min_depth, valid_fraction, gated=True)
        self._publish_stop(self._stop_state)
        return True

    def _publish_stop(self, stop: bool) -> None:
        self._stop_pub.publish(Bool(data=bool(stop)))

    def _maybe_log(
        self,
        min_depth: float,
        valid_fraction: float,
        gated: bool = False,
        insufficient: bool = False,
        altitude_gated: bool = False,
    ) -> None:
        if not self._log_min_depth:
            return

        now = self.get_clock().now()
        if (now - self._last_log_time).nanoseconds < int(self._log_period_s * 1e9):
            return
        self._last_log_time = now

        extras = []
        if gated:
            extras.append("gated_by_velocity")
        if insufficient:
            extras.append("insufficient_valid_pixels")
        if altitude_gated:
            extras.append("below_min_hold_altitude")

        v = self._motion.latest_forward_vel
        vtxt = "None" if v is None else f"{v:.2f}"
        alt = self._motion.latest_altitude_m
        alttxt = "None" if alt is None else f"{alt:.2f}"

        self.get_logger().info(
            f"min_depth={min_depth:.2f} m, valid_frac={valid_fraction:.3f}, "
            f"stop={self._stop_state}, v_fwd={vtxt} m/s, altitude={alttxt} m"
            + (f" [{', '.join(extras)}]" if extras else "")
        )

    def destroy_node(self) -> bool:
        self._measurement_map.shutdown()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = DepthStopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

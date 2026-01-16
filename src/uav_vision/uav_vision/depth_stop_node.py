#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Optional: velocity gating via PX4 or nav_msgs
# - If you do not have px4_msgs installed, disable px4 gating params or switch to nav_msgs/Odometry.
try:
    from px4_msgs.msg import VehicleOdometry
    PX4_MSGS_AVAILABLE = True
except Exception:
    PX4_MSGS_AVAILABLE = False
    VehicleOdometry = None  # type: ignore

try:
    from nav_msgs.msg import Odometry
    NAV_MSGS_AVAILABLE = True
except Exception:
    NAV_MSGS_AVAILABLE = False
    Odometry = None  # type: ignore


# Optional: direct PX4 control (Offboard setpoint / HOLD)
# This depends on your PX4-ROS2 bridge version and topic names.
try:
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
    PX4_OFFBOARD_AVAILABLE = True
except Exception:
    PX4_OFFBOARD_AVAILABLE = False
    OffboardControlMode = None  # type: ignore
    TrajectorySetpoint = None  # type: ignore
    VehicleCommand = None  # type: ignore


@dataclass(frozen=True)
class Roi:
    x0: int
    y0: int
    x1: int
    y1: int


class DepthStopNode(Node):
    """
    depth_stop_node:
      - Subscribes: depth_image (sensor_msgs/Image, expected 32FC1 in meters)
      - Optional gating: forward velocity (px4_msgs/VehicleOdometry or nav_msgs/Odometry)
      - Publishes: std_msgs/Bool /obstacle_stop
      - Optional: publishes PX4 offboard setpoints / HOLD (parameter-controlled)

    Detection approach:
      - Take ROI from depth map
      - Compute min depth (robustly, ignoring NaNs/inf/out-of-range)
      - Trigger stop when min_depth < threshold_m
      - Hysteresis via separate clear_threshold_m (optional)
      - Require minimum fraction of valid pixels in ROI
      - Optional: only trigger when forward velocity exceeds threshold (i.e., actually moving forward)
    """

    def __init__(self) -> None:
        super().__init__("depth_stop_node")
        self._bridge = CvBridge()

        # ---------------- Parameters ----------------
        self.declare_parameter("depth_topic", "/stereo/depth_left")
        self.declare_parameter("stop_topic", "/obstacle_stop")

        # ROI as fractions of image size
        # Example: focus on center-lower band where obstacles on flight path appear
        self.declare_parameter("roi_x_min", 0.35)  # fraction
        self.declare_parameter("roi_x_max", 0.65)  # fraction
        self.declare_parameter("roi_y_min", 0.40)  # fraction
        self.declare_parameter("roi_y_max", 0.85)  # fraction

        # Depth filtering
        self.declare_parameter("min_depth_m", 0.2)        # ignore anything closer than this (sensor artifacts)
        self.declare_parameter("max_depth_m", 20.0)       # ignore anything farther than this
        self.declare_parameter("threshold_m", 4.0)        # obstacle if min_depth < threshold_m
        self.declare_parameter("clear_threshold_m", 2.3)  # hysteresis: clear stop when min_depth > this
        self.declare_parameter("min_valid_fraction", 0.05)  # at least 5% valid pixels in ROI

        # Forward motion gating
        self.declare_parameter("use_velocity_gating", False)
        self.declare_parameter("velocity_source", "px4")  # "px4" or "nav"
        self.declare_parameter("velocity_topic", "/fmu/out/vehicle_odometry")  # for px4
        self.declare_parameter("forward_vel_threshold_mps", 0.2)  # consider "moving forward" if vx > this
        self.declare_parameter("forward_axis", "x")  # assumes body frame x forward; adjust if needed

        # Publish debugging info (logs)
        self.declare_parameter("log_min_depth", True)
        self.declare_parameter("log_period_s", 0.5)

        # Optional direct PX4 reaction
        self.declare_parameter("enable_px4_reaction", False)
        self.declare_parameter("px4_mode", "hold")  # "hold" or "offboard_zero_vel"
        self.declare_parameter("px4_offboard_rate_hz", 20.0)

        # Common PX4 topic defaults (may differ in your setup)
        self.declare_parameter("px4_offboard_mode_topic", "/fmu/in/offboard_control_mode")
        self.declare_parameter("px4_traj_sp_topic", "/fmu/in/trajectory_setpoint")
        self.declare_parameter("px4_vehicle_cmd_topic", "/fmu/in/vehicle_command")

        # For HOLD via VehicleCommand
        # These numeric values depend on px4_msgs definitions; we keep them configurable.
        self.declare_parameter("px4_cmd_nav_hold", 92)  # MAV_CMD_NAV_LOITER_UNLIM is 17 in MAVLink; PX4 bridges vary.
        self.declare_parameter("px4_cmd_custom", False)

        # ---------------- State ----------------
        self._stop_pub = self.create_publisher(Bool, self.get_parameter("stop_topic").value, 10)
        self._stop_state: bool = False

        self._latest_forward_vel: Optional[float] = None

        # Depth subscriber
        depth_topic = self.get_parameter("depth_topic").value
        self._depth_sub = self.create_subscription(Image, depth_topic, self._on_depth, 10)

        # Velocity subscriber (optional)
        self._vel_sub = None
        if bool(self.get_parameter("use_velocity_gating").value):
            self._init_velocity_subscription()

        # PX4 reaction (optional)
        self._px4_enabled = bool(self.get_parameter("enable_px4_reaction").value)
        self._px4_mode_pub = None
        self._px4_traj_pub = None
        self._px4_cmd_pub = None
        self._px4_timer = None
        if self._px4_enabled:
            self._init_px4_publishers_and_timer()

        # Logging throttle
        self._log_min_depth = bool(self.get_parameter("log_min_depth").value)
        self._log_period_s = float(self.get_parameter("log_period_s").value)
        self._last_log_time = self.get_clock().now()

        self.get_logger().info(
            f"DepthStopNode started. depth_topic={depth_topic}, stop_topic={self.get_parameter('stop_topic').value}"
        )

    # ---------------- Init helpers ----------------

    def _init_velocity_subscription(self) -> None:
        source = str(self.get_parameter("velocity_source").value).lower()
        vel_topic = str(self.get_parameter("velocity_topic").value)

        if source == "px4":
            if not PX4_MSGS_AVAILABLE:
                self.get_logger().error("use_velocity_gating=True but px4_msgs is not available.")
                return
            self._vel_sub = self.create_subscription(VehicleOdometry, vel_topic, self._on_px4_odometry, 10)
            self.get_logger().info(f"Velocity gating enabled (PX4): {vel_topic}")

        elif source == "nav":
            if not NAV_MSGS_AVAILABLE:
                self.get_logger().error("use_velocity_gating=True but nav_msgs is not available.")
                return
            self._vel_sub = self.create_subscription(Odometry, vel_topic, self._on_nav_odometry, 10)
            self.get_logger().info(f"Velocity gating enabled (nav_msgs): {vel_topic}")

        else:
            self.get_logger().error(f"Unsupported velocity_source='{source}'. Use 'px4' or 'nav'.")

    def _init_px4_publishers_and_timer(self) -> None:
        if not PX4_OFFBOARD_AVAILABLE:
            self.get_logger().error("enable_px4_reaction=True but px4 offboard msgs are not available.")
            self._px4_enabled = False
            return

        mode_topic = str(self.get_parameter("px4_offboard_mode_topic").value)
        sp_topic = str(self.get_parameter("px4_traj_sp_topic").value)
        cmd_topic = str(self.get_parameter("px4_vehicle_cmd_topic").value)

        self._px4_mode_pub = self.create_publisher(OffboardControlMode, mode_topic, 10)
        self._px4_traj_pub = self.create_publisher(TrajectorySetpoint, sp_topic, 10)
        self._px4_cmd_pub = self.create_publisher(VehicleCommand, cmd_topic, 10)

        rate_hz = float(self.get_parameter("px4_offboard_rate_hz").value)
        period = 1.0 / max(rate_hz, 1.0)
        self._px4_timer = self.create_timer(period, self._px4_timer_cb)

        self.get_logger().info(
            f"PX4 reaction enabled. mode={self.get_parameter('px4_mode').value}, "
            f"topics: {mode_topic}, {sp_topic}, {cmd_topic}"
        )

    # ---------------- Callbacks ----------------

    def _on_px4_odometry(self, msg: VehicleOdometry) -> None:
        # PX4 VehicleOdometry velocity is typically in body frame (depending on config).
        # Commonly: msg.velocity is [vx, vy, vz].
        axis = str(self.get_parameter("forward_axis").value).lower()
        idx = {"x": 0, "y": 1, "z": 2}.get(axis, 0)
        try:
            v = float(msg.velocity[idx])
            self._latest_forward_vel = v
        except Exception:
            self._latest_forward_vel = None

    def _on_nav_odometry(self, msg: Odometry) -> None:
        # nav_msgs/Odometry twist is in whatever frame your estimator uses; ensure it aligns with "forward".
        axis = str(self.get_parameter("forward_axis").value).lower()
        v = msg.twist.twist.linear
        self._latest_forward_vel = float({"x": v.x, "y": v.y, "z": v.z}.get(axis, v.x))

    def _on_depth(self, msg: Image) -> None:
        # Expect encoding 32FC1 (float32 meters). If your encoding differs, adapt accordingly.
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as exc:
            self.get_logger().error(f"Depth conversion failed: {exc}")
            return

        if depth.ndim != 2:
            self.get_logger().error(f"Expected 2D depth image, got shape={depth.shape}")
            return

        roi = self._compute_roi(depth.shape)
        depth_roi = depth[roi.y0:roi.y1, roi.x0:roi.x1].astype(np.float32)

        min_depth, valid_fraction = self._compute_min_depth(depth_roi)

        # Optional velocity gating
        if bool(self.get_parameter("use_velocity_gating").value):
            if not self._is_moving_forward():
                # If not moving forward, you can either keep state as-is or force clear.
                # Here: keep state as-is but do not trigger new stop from this frame.
                self._maybe_log(min_depth, valid_fraction, gated=True)
                self._publish_stop(self._stop_state)
                return

        # If not enough valid pixels, do not trigger stop (keeps last state)
        min_valid_fraction = float(self.get_parameter("min_valid_fraction").value)
        if valid_fraction < min_valid_fraction:
            self._maybe_log(min_depth, valid_fraction, gated=False, insufficient=True)
            self._publish_stop(self._stop_state)
            return

        threshold = float(self.get_parameter("threshold_m").value)
        clear_thr = float(self.get_parameter("clear_threshold_m").value)

        # Hysteresis
        if not self._stop_state:
            # set stop
            if np.isfinite(min_depth) and min_depth < threshold:
                self._stop_state = True
        else:
            # clear stop
            if np.isfinite(min_depth) and min_depth > clear_thr:
                self._stop_state = False

        self._maybe_log(min_depth, valid_fraction)
        self._publish_stop(self._stop_state)

    # ---------------- Core logic ----------------

    def _compute_roi(self, shape_hw: Tuple[int, int]) -> Roi:
        h, w = shape_hw
        x_min_f = float(self.get_parameter("roi_x_min").value)
        x_max_f = float(self.get_parameter("roi_x_max").value)
        y_min_f = float(self.get_parameter("roi_y_min").value)
        y_max_f = float(self.get_parameter("roi_y_max").value)

        x0 = int(np.clip(round(x_min_f * w), 0, w - 1))
        x1 = int(np.clip(round(x_max_f * w), x0 + 1, w))
        y0 = int(np.clip(round(y_min_f * h), 0, h - 1))
        y1 = int(np.clip(round(y_max_f * h), y0 + 1, h))
        return Roi(x0=x0, y0=y0, x1=x1, y1=y1)

    def _compute_min_depth(self, depth_roi: np.ndarray) -> Tuple[float, float]:
        min_depth_m = float(self.get_parameter("min_depth_m").value)
        max_depth_m = float(self.get_parameter("max_depth_m").value)

        # Valid pixels: finite and within range
        valid = np.isfinite(depth_roi) & (depth_roi >= min_depth_m) & (depth_roi <= max_depth_m)

        total = depth_roi.size
        valid_count = int(np.count_nonzero(valid))
        valid_fraction = (valid_count / total) if total > 0 else 0.0

        if valid_count == 0:
            return float("nan"), valid_fraction

        min_depth = float(np.min(depth_roi[valid]))
        return min_depth, valid_fraction

    def _is_moving_forward(self) -> bool:
        vthr = float(self.get_parameter("forward_vel_threshold_mps").value)
        v = self._latest_forward_vel
        if v is None:
            return False
        return v > vthr

    # ---------------- Publishing ----------------

    def _publish_stop(self, stop: bool) -> None:
        self._stop_pub.publish(Bool(data=bool(stop)))
        # Optional: if integrated PX4 reaction is enabled, timer will do the reaction.
        # We do not directly publish PX4 here to keep deterministic cadence.

    def _px4_timer_cb(self) -> None:
        if not self._px4_enabled:
            return

        mode = str(self.get_parameter("px4_mode").value).lower()
        now_us = int(self.get_clock().now().nanoseconds // 1000)

        if self._stop_state:
            if mode == "offboard_zero_vel":
                self._publish_px4_offboard_zero_vel(now_us)
            elif mode == "hold":
                self._publish_px4_hold_command(now_us)
        else:
            # When not stopping, do nothing. In a full system you may want to keep streaming offboard setpoints.
            pass

    def _publish_px4_offboard_zero_vel(self, now_us: int) -> None:
        # Streams OffboardControlMode + TrajectorySetpoint with zero velocity.
        # Note: Actual field names can differ between PX4 versions; adjust if needed.
        if self._px4_mode_pub is None or self._px4_traj_pub is None:
            return

        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        # Enable velocity control; disable position/acceleration unless you want them.
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self._px4_mode_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        # Depending on frame conventions in your PX4 config, these may be NED.
        # Zero velocity = "stop"; you may also want yaw hold.
        sp.vx = 0.0
        sp.vy = 0.0
        sp.vz = 0.0
        sp.yaw = float("nan")  # keep current yaw if supported
        self._px4_traj_pub.publish(sp)

    def _publish_px4_hold_command(self, now_us: int) -> None:
        # Sends a one-shot or repeated VehicleCommand to HOLD/LOITER.
        # IMPORTANT: The exact command ID depends on your integration. We keep it parameterized.
        if self._px4_cmd_pub is None:
            return

        cmd_id = int(self.get_parameter("px4_cmd_nav_hold").value)
        use_custom = bool(self.get_parameter("px4_cmd_custom").value)

        vc = VehicleCommand()
        vc.timestamp = now_us
        vc.param1 = 0.0
        vc.param2 = 0.0
        vc.param3 = 0.0
        vc.param4 = 0.0
        vc.param5 = 0.0
        vc.param6 = 0.0
        vc.param7 = 0.0

        # Typical PX4 fields; adjust target IDs to your setup.
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True

        if use_custom:
            vc.command = cmd_id
        else:
            # Use param as already set by cmd_id; no further assumptions.
            vc.command = cmd_id

        self._px4_cmd_pub.publish(vc)

    # ---------------- Logging ----------------

    def _maybe_log(
        self,
        min_depth: float,
        valid_fraction: float,
        gated: bool = False,
        insufficient: bool = False,
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

        v = self._latest_forward_vel
        vtxt = "None" if v is None else f"{v:.2f}"

        self.get_logger().info(
            f"min_depth={min_depth:.2f} m, valid_frac={valid_fraction:.3f}, "
            f"stop={self._stop_state}, v_fwd={vtxt} m/s"
            + (f" [{', '.join(extras)}]" if extras else "")
        )


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

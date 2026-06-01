from __future__ import annotations

from typing import Optional

try:
    from px4_msgs.msg import VehicleOdometry
    PX4_MSGS_AVAILABLE = True
except Exception:
    PX4_MSGS_AVAILABLE = False
    VehicleOdometry = None

try:
    from nav_msgs.msg import Odometry
    NAV_MSGS_AVAILABLE = True
except Exception:
    NAV_MSGS_AVAILABLE = False
    Odometry = None


def declare_motion_parameters(node) -> None:
    node.declare_parameter("use_velocity_gating", False)
    node.declare_parameter("velocity_source", "px4")
    node.declare_parameter("velocity_topic", "/fmu/out/vehicle_odometry")
    node.declare_parameter("forward_vel_threshold_mps", 0.2)
    node.declare_parameter("forward_axis", "x")

    node.declare_parameter("use_altitude_gating", True)
    node.declare_parameter("altitude_source", "px4")
    node.declare_parameter("altitude_topic", "/fmu/out/vehicle_odometry")
    node.declare_parameter("min_hold_altitude_m", 3.0)
    node.declare_parameter("altitude_axis", "z")
    node.declare_parameter("px4_altitude_is_ned", True)


class MotionState:
    def __init__(self, node, px4_qos_factory) -> None:
        self._node = node
        self._px4_qos_factory = px4_qos_factory
        self.latest_forward_vel: Optional[float] = None
        self.latest_altitude_m: Optional[float] = None
        self._vel_sub = None
        self._alt_sub = None

    def init_subscriptions(self) -> None:
        if bool(self._node.get_parameter("use_velocity_gating").value):
            self._init_velocity_subscription()
        if bool(self._node.get_parameter("use_altitude_gating").value):
            self._init_altitude_subscription()

    def is_moving_forward(self) -> bool:
        vthr = float(self._node.get_parameter("forward_vel_threshold_mps").value)
        if self.latest_forward_vel is None:
            return False
        return self.latest_forward_vel > vthr

    def is_below_hold_altitude(self) -> bool:
        if self.latest_altitude_m is None:
            return True
        return self.latest_altitude_m < float(self._node.get_parameter("min_hold_altitude_m").value)

    def _init_velocity_subscription(self) -> None:
        source = str(self._node.get_parameter("velocity_source").value).lower()
        vel_topic = str(self._node.get_parameter("velocity_topic").value)

        if source == "px4":
            if not PX4_MSGS_AVAILABLE:
                self._node.get_logger().error("use_velocity_gating=True but px4_msgs is not available.")
                return
            self._vel_sub = self._node.create_subscription(
                VehicleOdometry,
                vel_topic,
                self._on_px4_odometry,
                self._px4_qos_factory(),
            )
            self._node.get_logger().info(f"Velocity gating enabled (PX4): {vel_topic}")

        elif source == "nav":
            if not NAV_MSGS_AVAILABLE:
                self._node.get_logger().error("use_velocity_gating=True but nav_msgs is not available.")
                return
            self._vel_sub = self._node.create_subscription(Odometry, vel_topic, self._on_nav_odometry, 10)
            self._node.get_logger().info(f"Velocity gating enabled (nav_msgs): {vel_topic}")

        else:
            self._node.get_logger().error(f"Unsupported velocity_source='{source}'. Use 'px4' or 'nav'.")

    def _init_altitude_subscription(self) -> None:
        source = str(self._node.get_parameter("altitude_source").value).lower()
        alt_topic = str(self._node.get_parameter("altitude_topic").value)

        if source == "px4":
            if not PX4_MSGS_AVAILABLE:
                self._node.get_logger().error("use_altitude_gating=True but px4_msgs is not available.")
                return
            self._alt_sub = self._node.create_subscription(
                VehicleOdometry,
                alt_topic,
                self._on_px4_altitude,
                self._px4_qos_factory(),
            )
            self._node.get_logger().info(f"Altitude gating enabled (PX4): {alt_topic}")

        elif source == "nav":
            if not NAV_MSGS_AVAILABLE:
                self._node.get_logger().error("use_altitude_gating=True but nav_msgs is not available.")
                return
            self._alt_sub = self._node.create_subscription(Odometry, alt_topic, self._on_nav_altitude, 10)
            self._node.get_logger().info(f"Altitude gating enabled (nav_msgs): {alt_topic}")

        else:
            self._node.get_logger().error(f"Unsupported altitude_source='{source}'. Use 'px4' or 'nav'.")

    def _on_px4_odometry(self, msg: VehicleOdometry) -> None:
        axis = str(self._node.get_parameter("forward_axis").value).lower()
        idx = {"x": 0, "y": 1, "z": 2}.get(axis, 0)
        try:
            self.latest_forward_vel = float(msg.velocity[idx])
        except Exception:
            self.latest_forward_vel = None
        self._update_px4_altitude(msg)

    def _on_nav_odometry(self, msg: Odometry) -> None:
        axis = str(self._node.get_parameter("forward_axis").value).lower()
        v = msg.twist.twist.linear
        self.latest_forward_vel = float({"x": v.x, "y": v.y, "z": v.z}.get(axis, v.x))
        self._update_nav_altitude(msg)

    def _on_px4_altitude(self, msg: VehicleOdometry) -> None:
        self._update_px4_altitude(msg)

    def _on_nav_altitude(self, msg: Odometry) -> None:
        self._update_nav_altitude(msg)

    def _update_px4_altitude(self, msg: VehicleOdometry) -> None:
        axis = str(self._node.get_parameter("altitude_axis").value).lower()
        idx = {"x": 0, "y": 1, "z": 2}.get(axis, 2)
        try:
            position = float(msg.position[idx])
        except Exception:
            self.latest_altitude_m = None
            return

        if idx == 2 and bool(self._node.get_parameter("px4_altitude_is_ned").value):
            position = -position
        if idx == 2:
            position = abs(position)
        self.latest_altitude_m = position

    def _update_nav_altitude(self, msg: Odometry) -> None:
        axis = str(self._node.get_parameter("altitude_axis").value).lower()
        p = msg.pose.pose.position
        self.latest_altitude_m = float({"x": p.x, "y": p.y, "z": p.z}.get(axis, p.z))

from __future__ import annotations

try:
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
    PX4_OFFBOARD_AVAILABLE = True
except Exception:
    PX4_OFFBOARD_AVAILABLE = False
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommand = None


def declare_px4_reaction_parameters(node) -> None:
    node.declare_parameter("enable_px4_reaction", False)
    node.declare_parameter("px4_mode", "hold")
    node.declare_parameter("px4_offboard_rate_hz", 20.0)

    node.declare_parameter("px4_offboard_mode_topic", "/fmu/in/offboard_control_mode")
    node.declare_parameter("px4_traj_sp_topic", "/fmu/in/trajectory_setpoint")
    node.declare_parameter("px4_vehicle_cmd_topic", "/fmu/in/vehicle_command")

    node.declare_parameter("px4_cmd_nav_hold", 92)
    node.declare_parameter("px4_cmd_custom", False)


class Px4Reaction:
    def __init__(self, node, stop_state_fn) -> None:
        self._node = node
        self._stop_state_fn = stop_state_fn
        self.enabled = bool(node.get_parameter("enable_px4_reaction").value)
        self._mode_pub = None
        self._traj_pub = None
        self._cmd_pub = None
        self._timer = None

        if self.enabled:
            self._init_publishers_and_timer()

    def _init_publishers_and_timer(self) -> None:
        if not PX4_OFFBOARD_AVAILABLE:
            self._node.get_logger().error("enable_px4_reaction=True but px4 offboard msgs are not available.")
            self.enabled = False
            return

        mode_topic = str(self._node.get_parameter("px4_offboard_mode_topic").value)
        sp_topic = str(self._node.get_parameter("px4_traj_sp_topic").value)
        cmd_topic = str(self._node.get_parameter("px4_vehicle_cmd_topic").value)

        self._mode_pub = self._node.create_publisher(OffboardControlMode, mode_topic, 10)
        self._traj_pub = self._node.create_publisher(TrajectorySetpoint, sp_topic, 10)
        self._cmd_pub = self._node.create_publisher(VehicleCommand, cmd_topic, 10)

        rate_hz = float(self._node.get_parameter("px4_offboard_rate_hz").value)
        period = 1.0 / max(rate_hz, 1.0)
        self._timer = self._node.create_timer(period, self._timer_cb)

        self._node.get_logger().info(
            f"PX4 reaction enabled. mode={self._node.get_parameter('px4_mode').value}, "
            f"topics: {mode_topic}, {sp_topic}, {cmd_topic}"
        )

    def _timer_cb(self) -> None:
        if not self.enabled or not self._stop_state_fn():
            return

        mode = str(self._node.get_parameter("px4_mode").value).lower()
        now_us = int(self._node.get_clock().now().nanoseconds // 1000)

        if mode == "offboard_zero_vel":
            self._publish_offboard_zero_vel(now_us)
        elif mode == "hold":
            self._publish_hold_command(now_us)

    def _publish_offboard_zero_vel(self, now_us: int) -> None:
        if self._mode_pub is None or self._traj_pub is None:
            return

        ocm = OffboardControlMode()
        ocm.timestamp = now_us
        ocm.position = False
        ocm.velocity = True
        ocm.acceleration = False
        ocm.attitude = False
        ocm.body_rate = False
        self._mode_pub.publish(ocm)

        sp = TrajectorySetpoint()
        sp.timestamp = now_us
        sp.vx = 0.0
        sp.vy = 0.0
        sp.vz = 0.0
        sp.yaw = float("nan")
        self._traj_pub.publish(sp)

    def _publish_hold_command(self, now_us: int) -> None:
        if self._cmd_pub is None:
            return

        vc = VehicleCommand()
        vc.timestamp = now_us
        vc.param1 = 0.0
        vc.param2 = 0.0
        vc.param3 = 0.0
        vc.param4 = 0.0
        vc.param5 = 0.0
        vc.param6 = 0.0
        vc.param7 = 0.0

        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        vc.from_external = True
        vc.command = int(self._node.get_parameter("px4_cmd_nav_hold").value)

        self._cmd_pub.publish(vc)

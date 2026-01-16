#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


try:
    from px4_msgs.msg import VehicleCommand
    PX4_MSGS_AVAILABLE = True
except Exception:
    PX4_MSGS_AVAILABLE = False
    VehicleCommand = None  # type: ignore


@dataclass(frozen=True)
class Px4Mode:
    custom_main_mode: int
    custom_sub_mode: int


class StopControllerNode(Node):
    """
    stop_controller_node:
      - Subscribes: /obstacle_stop (std_msgs/Bool)
      - On True (rising edge): sends PX4 VehicleCommand to switch flight mode to HOLD/LOITER
      - On False: does nothing

    Rationale:
      - During manual control from QGC/RC, publishing setpoints from ROS often has no effect.
        Forcing a HOLD/LOITER mode is a practical, widely used safety reaction.
    """

    def __init__(self) -> None:
        super().__init__("stop_controller_node")

        if not PX4_MSGS_AVAILABLE:
            raise RuntimeError("px4_msgs is required for StopControllerNode (VehicleCommand).")

        # -------- Parameters --------
        self.declare_parameter("obstacle_stop_topic", "/obstacle_stop")
        self.declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command")

        # Action mode:
        #  - "hold": AUTO+LOITER (PX4 Hold/Loiter)
        #  - "posctl": switch to Position Control (manual-assist, also effectively brakes when sticks centered)
        self.declare_parameter("action_mode", "hold")  # "hold" or "posctl"

        # Reliability / rate limiting
        self.declare_parameter("cooldown_s", 1.0)      # minimum time between re-sends
        self.declare_parameter("retries", 3)           # how many times to send the mode command on trigger
        self.declare_parameter("retry_period_s", 0.1)  # spacing between retries

        # System/component IDs (adjust if needed)
        self.declare_parameter("target_system", 1)
        self.declare_parameter("target_component", 1)
        self.declare_parameter("source_system", 1)
        self.declare_parameter("source_component", 1)

        # -------- State --------
        self._stop_prev: bool = False
        self._last_send_ns: int = 0

        self._pending_retries: int = 0
        self._retry_timer = None

        # -------- ROS interfaces --------
        vehicle_command_topic = self.get_parameter("vehicle_command_topic").value

        qos_px4_in = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._cmd_pub = self.create_publisher(
            VehicleCommand,
            vehicle_command_topic,
            qos_px4_in
        )

        self._stop_sub = self.create_subscription(Bool, self.get_parameter("obstacle_stop_topic").value, self._on_stop, 10)

        self.get_logger().info(
            f"StopControllerNode started. obstacle_stop_topic={self.get_parameter('obstacle_stop_topic').value}, "
            f"vehicle_command_topic={self.get_parameter('vehicle_command_topic').value}, "
            f"action_mode={self.get_parameter('action_mode').value}"
        )

    def _on_stop(self, msg: Bool) -> None:
        stop = bool(msg.data)

        # Only react on rising edge (False -> True)
        if stop and not self._stop_prev:
            if self._cooldown_ok():
                self.get_logger().warn("Obstacle STOP=True: sending PX4 mode change command (HOLD/LOITER or POSCTL).")
                self._start_retries()
            else:
                self.get_logger().info("STOP trigger ignored due to cooldown.")
        # When stop becomes False: do nothing (control returns to operator)
        self._stop_prev = stop

    # ---------------- Mode command logic ----------------

    def _cooldown_ok(self) -> bool:
        cooldown_s = float(self.get_parameter("cooldown_s").value)
        now_ns = self.get_clock().now().nanoseconds
        if self._last_send_ns == 0:
            return True
        return (now_ns - self._last_send_ns) >= int(cooldown_s * 1e9)

    def _start_retries(self) -> None:
        self._pending_retries = int(self.get_parameter("retries").value)
        period_s = float(self.get_parameter("retry_period_s").value)
        self._last_send_ns = self.get_clock().now().nanoseconds

        # Send immediately once, then schedule remaining retries (if any)
        self._send_mode_command_once()
        self._pending_retries -= 1

        if self._pending_retries > 0:
            if self._retry_timer is not None:
                self._retry_timer.cancel()
            self._retry_timer = self.create_timer(period_s, self._retry_tick)

    def _retry_tick(self) -> None:
        if self._pending_retries <= 0:
            if self._retry_timer is not None:
                self._retry_timer.cancel()
                self._retry_timer = None
            return

        self._send_mode_command_once()
        self._pending_retries -= 1

    def _send_mode_command_once(self) -> None:
        mode = self._selected_mode()

        now_us = int(self.get_clock().now().nanoseconds // 1000)

        cmd = VehicleCommand()
        cmd.timestamp = now_us

        # VEHICLE_CMD_DO_SET_MODE = 176
        cmd.command = 176

        # param1: base_mode flag. For PX4 custom modes commonly:
        # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
        # We set param1=1 to indicate custom modes are used.
        cmd.param1 = 1.0

        # param2/param3: PX4 custom main/sub mode
        cmd.param2 = float(mode.custom_main_mode)
        cmd.param3 = float(mode.custom_sub_mode)

        cmd.param4 = 0.0
        cmd.param5 = 0.0
        cmd.param6 = 0.0
        cmd.param7 = 0.0

        cmd.target_system = int(self.get_parameter("target_system").value)
        cmd.target_component = int(self.get_parameter("target_component").value)
        cmd.source_system = int(self.get_parameter("source_system").value)
        cmd.source_component = int(self.get_parameter("source_component").value)
        cmd.from_external = True

        self._cmd_pub.publish(cmd)

    def _selected_mode(self) -> Px4Mode:
        action_mode = str(self.get_parameter("action_mode").value).strip().lower()

        # Values based on common PX4 custom mode mapping:
        # AUTO=4, AUTO_LOITER=3, POSCTL=3 (main mode) with submode 0
        # (See PX4 custom mode definitions in mavutil.py mapping.) :contentReference[oaicite:2]{index=2}
        if action_mode == "hold":
            # Hold/Loiter = AUTO + LOITER
            return Px4Mode(custom_main_mode=4, custom_sub_mode=3)
        elif action_mode == "posctl":
            # Position Control (manual assist). Often brakes when sticks are centered.
            return Px4Mode(custom_main_mode=3, custom_sub_mode=0)
        else:
            self.get_logger().warn(f"Unknown action_mode='{action_mode}', defaulting to 'hold'.")
            return Px4Mode(custom_main_mode=4, custom_sub_mode=3)


def main() -> None:
    rclpy.init()
    node = StopControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

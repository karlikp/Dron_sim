import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleAttitude
from std_msgs.msg import Float64


class GimbalPhotoController(Node):
    """Keep the simulated nadir camera world-stabilized."""

    def __init__(self):
        super().__init__('gimbal_photo_controller')

        self.roll_offset_rad = float(self.declare_parameter('roll_offset_rad', 0.0).value)
        self.pitch_offset_rad = float(self.declare_parameter('pitch_offset_rad', 0.0).value)
        self.yaw_offset_rad = float(self.declare_parameter('yaw_offset_rad', 0.0).value)
        self.compensate_yaw = bool(self.declare_parameter('compensate_yaw', True).value)
        self.command_rate_hz = float(self.declare_parameter('command_rate_hz', 30.0).value)

        if self.command_rate_hz <= 0.0:
            self.get_logger().warn('command_rate_hz must be > 0. Falling back to 30 Hz.')
            self.command_rate_hz = 30.0

        self.roll_pub = self.create_publisher(Float64, '/gimbal/roll_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/gimbal/pitch_cmd', 10)
        self.yaw_pub = self.create_publisher(Float64, '/gimbal/yaw_cmd', 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_cb,
            px4_qos,
        )

        self.roll_cmd = self.roll_offset_rad
        self.pitch_cmd = self.pitch_offset_rad
        self.yaw_cmd = self.yaw_offset_rad
        self.timer = self.create_timer(1.0 / self.command_rate_hz, self.timer_cb)
        self.get_logger().info(
            'Nadir gimbal stabilization enabled: '
            f'roll_offset={self.roll_offset_rad:.3f}, '
            f'pitch_offset={self.pitch_offset_rad:.3f}, '
            f'yaw_offset={self.yaw_offset_rad:.3f} rad'
        )

    @staticmethod
    def clamp(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def wrap_pi(value):
        return math.atan2(math.sin(value), math.cos(value))

    @staticmethod
    def matmul(a, b):
        return [
            [
                sum(a[row][idx] * b[idx][col] for idx in range(3))
                for col in range(3)
            ]
            for row in range(3)
        ]

    @staticmethod
    def transpose(m):
        return [[m[col][row] for col in range(3)] for row in range(3)]

    @staticmethod
    def zxy_to_matrix(yaw, roll, pitch):
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)

        return [
            [cp * cy - sp * sr * sy, -cr * sy, sp * cy + cp * sr * sy],
            [cp * sy + sp * sr * cy, cr * cy, sp * sy - cp * sr * cy],
            [-sp * cr, sr, cp * cr],
        ]

    @classmethod
    def matrix_to_zxy(cls, matrix):
        roll = math.asin(cls.clamp(matrix[2][1], -1.0, 1.0))
        cos_roll = math.cos(roll)

        if abs(cos_roll) < 1e-6:
            yaw = 0.0
            pitch = math.atan2(matrix[0][2], matrix[0][0])
        else:
            yaw = math.atan2(-matrix[0][1], matrix[1][1])
            pitch = math.atan2(-matrix[2][0], matrix[2][2])

        return yaw, roll, pitch

    @staticmethod
    def quaternion_to_matrix(q):
        w, x, y, z = q
        norm = math.sqrt(w * w + x * x + y * y + z * z)
        if norm == 0.0:
            return [
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ]

        w /= norm
        x /= norm
        y /= norm
        z /= norm

        return [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]

    def attitude_cb(self, msg):
        ned_from_frd = self.quaternion_to_matrix(msg.q)

        # PX4 publishes FRD-in-NED attitude. Gazebo joints are FLU-in-ENU, so
        # convert frames before solving the Z-X-Y gimbal chain.
        px4_to_gz = [
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0],
        ]
        enu_from_flu = self.matmul(self.matmul(px4_to_gz, ned_from_frd), px4_to_gz)

        target_world = self.zxy_to_matrix(
            self.yaw_offset_rad if self.compensate_yaw else 0.0,
            self.roll_offset_rad,
            self.pitch_offset_rad,
        )
        gimbal_target = self.matmul(self.transpose(enu_from_flu), target_world)
        yaw, roll, pitch = self.matrix_to_zxy(gimbal_target)

        self.roll_cmd = self.clamp(roll, -1.5708, 1.5708)
        self.pitch_cmd = self.clamp(pitch, -1.5708, 1.5708)
        if self.compensate_yaw:
            self.yaw_cmd = self.wrap_pi(yaw)
        else:
            self.yaw_cmd = self.yaw_offset_rad

    def timer_cb(self):
        self.roll_pub.publish(Float64(data=self.roll_cmd))
        self.pitch_pub.publish(Float64(data=self.pitch_cmd))
        self.yaw_pub.publish(Float64(data=self.yaw_cmd))


def main(args=None):
    rclpy.init(args=args)
    node = GimbalPhotoController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

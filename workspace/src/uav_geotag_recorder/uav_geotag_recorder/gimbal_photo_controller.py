import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GimbalPhotoController(Node):
    """Command the simulated gimbal to a fixed attitude."""

    def __init__(self):
        super().__init__('gimbal_photo_controller')

        self.roll_rad = float(self.declare_parameter('roll_rad', 0.0).value)
        self.pitch_rad = float(self.declare_parameter('pitch_rad', 1.5708).value)
        self.yaw_rad = float(self.declare_parameter('yaw_rad', 0.0).value)
        self.command_rate_hz = float(self.declare_parameter('command_rate_hz', 2.0).value)

        if self.command_rate_hz <= 0.0:
            self.get_logger().warn('command_rate_hz must be > 0. Falling back to 2 Hz.')
            self.command_rate_hz = 2.0

        self.roll_pub = self.create_publisher(Float64, '/gimbal/roll_cmd', 10)
        self.pitch_pub = self.create_publisher(Float64, '/gimbal/pitch_cmd', 10)
        self.yaw_pub = self.create_publisher(Float64, '/gimbal/yaw_cmd', 10)

        self.timer = self.create_timer(1.0 / self.command_rate_hz, self.timer_cb)
        self.get_logger().info(
            'Gimbal target: '
            f'roll={self.roll_rad:.3f}, pitch={self.pitch_rad:.3f}, yaw={self.yaw_rad:.3f} rad'
        )

    def timer_cb(self):
        self.roll_pub.publish(Float64(data=self.roll_rad))
        self.pitch_pub.publish(Float64(data=self.pitch_rad))
        self.yaw_pub.publish(Float64(data=self.yaw_rad))


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

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockListener(Node):
    def __init__(self):
        super().__init__('clock_listener')
        self.subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            10)
        self.subscription  # zapobiega ostrzeżeniu o nieużyciu zmiennej

    def clock_callback(self, msg):
        self.get_logger().info(f'Czas symulacji: {msg.clock.sec}.{msg.clock.nanosec:09d}')

def main(args=None):
    rclpy.init(args=args)
    node = ClockListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

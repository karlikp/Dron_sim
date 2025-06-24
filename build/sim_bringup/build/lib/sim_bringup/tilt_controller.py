import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty
import select
import time

def get_key(timeout=0.1):
    """Funkcja do czytania jednego znaku z klawiatury, bez blokowania."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class TiltController(Node):
    def __init__(self):
        super().__init__('tilt_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/oakd_tilt_position_controller/joint_trajectory', 10)
        self.angle = 0.0
        self.step = 0.05        # krok regulacji (radiany)
        self.min_angle = -1.57  # min kąt (dół)
        self.max_angle = 1.57   # max kąt (góra)
        self.send_angle()

    def send_angle(self):
        msg = JointTrajectory()
        msg.joint_names = ['oakd_tilt_joint']
        point = JointTrajectoryPoint()
        point.positions = [self.angle]
        point.time_from_start.sec = 1
        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Ustawiono kąt gimbala: {self.angle:.2f} rad')

def main(args=None):
    rclpy.init(args=args)
    node = TiltController()
    print("Sterowanie gimbalem: Q = w górę, Z = w dół, X = wyjście")

    try:
        while rclpy.ok():
            key = get_key()
            if key.upper() == 'Q':
                node.angle += node.step
                if node.angle > node.max_angle:
                    node.angle = node.max_angle
                node.send_angle()
            elif key.upper() == 'Z':
                node.angle -= node.step
                if node.angle < node.min_angle:
                    node.angle = node.min_angle
                node.send_angle()
            elif key.upper() == 'X':
                print("Koniec programu.")
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

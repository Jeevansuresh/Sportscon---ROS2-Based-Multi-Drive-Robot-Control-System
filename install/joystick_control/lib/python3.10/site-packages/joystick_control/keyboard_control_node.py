import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import tty
import termios


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Keyboard Control Node is running. Use W/A/S/D keys to move. Press Q to quit.')

        # Velocity parameters
        self.linear_speed = 0.5
        self.angular_speed = 1.0

    def get_key(self):
        """Waits for a single key press without Enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        while rclpy.ok():
            key = self.get_key()
            twist = Twist()

            # Velocity command assignments
            if key.lower() == 'w':
                twist.linear.x = self.linear_speed
            elif key.lower() == 's':
                twist.linear.x = -self.linear_speed
            elif key.lower() == 'a':
                twist.angular.z = self.angular_speed
            elif key.lower() == 'd':
                twist.angular.z = -self.angular_speed
            elif key.lower() == 'q':
                self.get_logger().info("Exit requested. Goodbye!")
                break
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # Publish Twist command
            self.pub.publish(twist)
            self.get_logger().info(
                f"Published Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

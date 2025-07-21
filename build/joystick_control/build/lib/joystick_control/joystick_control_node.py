import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info("Joystick Control Node publishing linear and angular velocities.")

    def joy_callback(self, msg):
        twist = Twist()
        # Adjust these indices for your joystick model
        twist.linear.x = msg.axes[1]    # Typically left stick vertical for forward/backward
        twist.angular.z = msg.axes[0]   # Typically left stick horizontal for turning
        self.pub.publish(twist)
        self.get_logger().info(
            f"Published Twist: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = JoystickControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

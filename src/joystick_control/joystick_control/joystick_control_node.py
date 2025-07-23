import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.get_logger().info("Joystick Control Node using buttons for movement.")

        self.linear_speed = 0.5   # m/s
        self.angular_speed = 1.0  # rad/s

    def joy_callback(self, msg):
        twist = Twist()

        # Button mapping (adjust as needed)
        forward    = msg.buttons[0]  # A / X
        backward   = msg.buttons[1]  # B / Circle
        turn_left  = msg.buttons[4]  # LB / L1
        turn_right = msg.buttons[5]  # RB / R1

        if forward:
            twist.linear.x = self.linear_speed
        elif backward:
            twist.linear.x = -self.linear_speed

        if turn_left:
            twist.angular.z = self.angular_speed
        elif turn_right:
            twist.angular.z = -self.angular_speed

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

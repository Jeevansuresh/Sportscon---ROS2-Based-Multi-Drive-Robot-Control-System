import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')

        # Subscribe to cmd_vel
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)

        # Publisher for PWM values (for all 4 wheels)
        self.pwm_pub = self.create_publisher(Float32MultiArray, 'wheel_pwm', 10)

        # Robot parameters
        self.wheel_base = 0.3   # Distance between wheels (meters)
        self.pwm_max = 255.0    # Max PWM output

        self.get_logger().info("Differential Drive Node is running and subscribed to /cmd_vel")

    def cmd_callback(self, msg):
        v = msg.linear.x        # Forward/backward velocity (m/s)
        omega = msg.angular.z   # Angular velocity (rad/s)

        # Compute individual wheel velocities (simplified differential drive formula)
        v_r = v + (omega * self.wheel_base / 2)
        v_l = v - (omega * self.wheel_base / 2)

        # Convert to PWM by assuming max speed maps to max PWM
        # We'll treat v = 1.0 m/s → 255 PWM
        pwm_r = v_r * self.pwm_max
        pwm_l = v_l * self.pwm_max

        # Clip to valid PWM range
        pwm_r = max(min(pwm_r, self.pwm_max), -self.pwm_max)
        pwm_l = max(min(pwm_l, self.pwm_max), -self.pwm_max)

        # Publish for 4 wheels: LF, RF, LR, RR
        pwm_msg = Float32MultiArray()
        pwm_msg.data = [pwm_l, pwm_r, pwm_l, pwm_r]

        self.pwm_pub.publish(pwm_msg)

        self.get_logger().info(
            f"cmd_vel -> linear.x={v:.2f}, angular.z={omega:.2f} -> PWM L={pwm_l:.1f}, R={pwm_r:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

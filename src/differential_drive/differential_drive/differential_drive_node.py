import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pwm_pub = self.create_publisher(Float32MultiArray, 'wheel_pwm', 10)

        self.wheel_base = 0.3   # meters
        self.pwm_max = 255.0    # max PWM
        self.speed_max = 1.0    # max velocity in m/s assumed for mapping

        self.get_logger().info("Differential Drive Node is running and subscribed to /cmd_vel")

    def map_value(self, x, in_min, in_max, out_min, out_max):
        # Clamp input first
        x = max(min(x, in_max), in_min)
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z

        # Compute wheel linear velocities
        v_r = v + (omega * self.wheel_base / 2)
        v_l = v - (omega * self.wheel_base / 2)

        # Map velocities to PWM
        pwm_r = self.map_value(v_r, -self.speed_max, self.speed_max, -self.pwm_max, self.pwm_max)
        pwm_l = self.map_value(v_l, -self.speed_max, self.speed_max, -self.pwm_max, self.pwm_max)

        pwm_msg = Float32MultiArray()
        pwm_msg.data = [pwm_l, pwm_r, pwm_l, pwm_r]  # LF, RF, LR, RR

        self.pwm_pub.publish(pwm_msg)

        self.get_logger().info(
            f"linear.x={v:.2f}, angular.z={omega:.2f} -> PWM L={pwm_l:.1f}, R={pwm_r:.1f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

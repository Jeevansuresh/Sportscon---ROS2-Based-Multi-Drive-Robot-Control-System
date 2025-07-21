import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DifferentialDriveNode(Node):
    def __init__(self):
        super().__init__('differential_drive_node')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        # Initialize hardware interface / PWM publisher here

        self.L = 0.3  # wheel base (distance between wheels, meters)
    
    def cmd_callback(self, msg):
        v = msg.linear.x
        omega = msg.angular.z
        v_r = v + (omega * self.L / 2)
        v_l = v - (omega * self.L / 2)
        # Convert v_r, v_l to PWM for your motor controller
        # pwm_r = velocity_to_pwm(v_r)
        # pwm_l = velocity_to_pwm(v_l)
        # Publish or output to hardware here
        self.get_logger().info(f"Right wheel: {v_r:.2f}, Left wheel: {v_l:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = DifferentialDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import numpy as np
class MecanumDriveNode(Node):
    def __init__(self):
        super().__init__('mecanum_drive_node')
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, 'wheel_speeds', 10)
        self.Lx = 0.25   
        self.Ly = 0.25   
        self.r = 0.05    
    def cmd_callback(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z
        self.get_logger().info(f"Received cmd_vel: vx={vx}, vy={vy}, wz={wz}")
        J = np.array([
            [1, -1, -(self.Lx + self.Ly)], 
            [1,  1,  (self.Lx + self.Ly)],  
            [1,  1, -(self.Lx + self.Ly)],  
            [1, -1,  (self.Lx + self.Ly)]   
        ])
        v = np.array([vx, vy, wz])
        omega = (1 / self.r) * (J @ v)
        msg_out = Float64MultiArray()
        msg_out.data = omega.tolist()
        self.pub.publish(msg_out)
        self.get_logger().info(f"Wheel Speeds: {omega}")
def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

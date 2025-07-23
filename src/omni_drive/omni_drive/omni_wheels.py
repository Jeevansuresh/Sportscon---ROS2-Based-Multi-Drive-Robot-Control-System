import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# vx --> velocity in x (forward and backward)
# vy --> vel in y (left and right)
# omega --> rotation arnd z

# for better understanding and debuging --> https://youtu.be/-wzl8XJopgg

class Omni_Wheels(Node):
    def __init__(self):
        super().__init__("omni_wheel")

        self.subscribe = self.create_subscription(Twist , 'cmd_vel' , self.callback , 10)

        self.pwm_pub = self.create_publisher(Float32MultiArray , "pwm_omni_wheels" , 10) # corrected the message type

        # Robot parameters
        self.robot_length = 0.25  # meters: Length of the robot (distance between wheels along length)
        self.wheel_base = 0.3   # meters: Distance between wheels (width)
        self.radius = 0.5       # meters: radius of the wheels
        self.pwm_max = 255.0    # Max PWM output

        self.get_logger().info("omni node is running !!!!!!!!!")

    def callback(self , msg):
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        pwm_max = self.pwm_max


        R = self.radius
        L = self.robot_length/2
        W = self.wheel_base/2

        v1 = vx - vy - (L + W) * omega  # FL
        v2 = vx + vy + (L + W) * omega  # FR
        v3 = vx + vy - (L + W) * omega  # RL
        v4 = vx - vy + (L + W) * omega  # RR

        pwms = [v*pwm_max for v in [v1 , v2 , v3, v4]]
        pwms = [max(min(p , pwm_max) , -pwm_max) for p in pwms]

        pwm_msg = Float32MultiArray()
        pwm_msg.data = pwms
        self.pwm_pub.publish(pwm_msg)

        self.get_logger().info(
            f"cmd_vel --> linear.x = {vx:.2f} , linear.y = {vy:.2f} , angular.z(omega) = {omega:.2f} ---> pwm: Front_Left = {pwms[0]} , Front_Right = {pwms[1]}, Rear_Left = {pwms[2]} , Rear_Right = {pwms[3]}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = Omni_Wheels()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
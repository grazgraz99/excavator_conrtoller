import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class MultiplexerNode(Node):
    def __init__(self):
        super().__init__('multiplexer')
        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.pure_pursuit_sub = self.create_subscription(Twist, 'fake_cmd_vel', self.pure_pursuit_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.use_joystick = True  # Default to joystick control
        
        
        self.joy_timeout=3 #number of seconds from last joy input, 
        #after which-other controller is used

        self.joy_cmd = Twist()
        self.pure_pursuit_cmd = Twist()
        self.last_joy_time=self.get_clock().now()

    def joy_callback(self, msg):
        self.use_joystick=True
        self.joy_cmd.linear.x = msg.axes[1]
        self.joy_cmd.angular.z = msg.axes[0]
        if self.use_joystick:
            self.last_joy_time = self.get_clock().now()
            self.cmd_pub.publish(self.joy_cmd)
            

    def pure_pursuit_callback(self, msg):
        self.check_timeout()
        self.pure_pursuit_cmd = msg
        if not self.use_joystick:
            self.cmd_pub.publish(self.pure_pursuit_cmd)

    def check_timeout(self):
        if self.use_joystick:
            now = self.get_clock().now()
            if (now - self.last_joy_time).nanoseconds * 1e-9 > self.joy_timeout:
                self.use_joystick = False
                #self.cmd_pub.publish(self.pure_pursuit_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MultiplexerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
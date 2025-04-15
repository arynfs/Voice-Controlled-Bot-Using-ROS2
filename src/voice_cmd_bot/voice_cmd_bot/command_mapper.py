import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class CommandMapperNode(Node):
    def __init__(self):
        super().__init__('command_mapper')
        self.subscription = self.create_subscription(String, 'voice_commands', self.listener_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("CommandMapperNode ready.")

    def listener_callback(self, msg):
        cmd = msg.data.lower()
        twist = Twist()

        if "forward" in cmd:
            twist.linear.x = 0.2
        elif "backward" in cmd:
            twist.linear.x = -0.2
        elif "left" in cmd:
            twist.angular.z = 0.5
        elif "right" in cmd:
            twist.angular.z = -0.5
        elif "stop" in cmd:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().info(f"Ignored: {cmd}")
            return

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Sent movement command for: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = CommandMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

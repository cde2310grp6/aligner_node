import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
import ast

class Aligner(Node):
    def __init__(self):
        super().__init__('aligner_node')

        self.ir_sub = self.create_subscription(
            String,
            '/ir_data',
            self.aligner_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist, 
            '/cmd_vel',
            10
        )

        self.target_index = 4
        self.last_max_index = None

    
    def aligner_callback(self, msg):
        threshold = 25
        values = ast.literal_eval(msg.data)
        self.get_logger().info(f'Received IR values: {values}')
        if len(values) != 8:
            self.get_logger().warn('Invalid data received: length is not 8')
        
        max_value = max(values)
        max_index = values.index(max_value)

        if max_value > threshold:
            self.get_logger().info(f'Max reading above threshold at {max_index} is {max_value}')

            if max_index == self.target_index or max_index == (self.target_index + 1) or max_index == (self.target_index - 1):
                self.stop()
            else:
                direction = self.get_rotation_direction(max_index)
                self.rotate(direction)


    def get_rotation_direction(self, max_index):
        if max_index < 4:
            return 1
        else:
            return -1

    def rotate(self, direction):
        twist = Twist()
        twist.angular.z = 0.3 * direction
        self.cmd_pub.publish(twist)

    def stop(self):
        twist = Twist()
        self.cmd_pub.publish(twist)
        self.get_logger().info('Aligned with heat source')

def main(args=None):
    rclpy.init(args=args)
    node = Aligner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

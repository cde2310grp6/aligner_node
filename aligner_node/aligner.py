import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist 
import ast
import numpy as np
from rclpy.qos import qos_profile_sensor_data

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

        self.aligner_complete_pub = self.create_publisher(
            Bool,
            '/align_complete',
            10
        )

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.srv = self.create_service(Trigger, 'aligner_service_call', self.align_service_callback)

        self.align_complete = False
        self.align_now = False
        self.target_index = 4
        self.last_max_index = None
        self.converting = False

        self.rotation_start_time = None
        self.rotation_timeout = 22
    
    def scan_callback(self, msg):
        if self.converting == True:
            return
        self.converting = True
        self.distances = np.array(msg.ranges)
        self.distances[self.distances==0] = np.nan
        self.distance_at_zero = 0
        count = 0
        swerve = []
        for i in range(-90, 91):
            if not np.isnan(self.distances[i]):
                swerve.append(self.distances[i])
        swerve1 = sorted(swerve)
        self.distance_at_zero = np.average([swerve1[1], swerve1[2], swerve1[3]])


        # self.get_logger().info(f'Distance in front of robot is {self.distance_at_zero}')
        self.converting = False

    
    def aligner_callback(self, msg):
        current_time = self.get_clock().now()
        if not self.align_now:
            return
        
        values = ast.literal_eval(msg.data)

        threshold_dynamic = np.average(values) + 2
        threshold = 32
        threshold2 = 0.3
        
        self.get_logger().info(f'Received IR values: {values}')
        if len(values) != 8:
            self.get_logger().warn('Invalid data received: length is not 8')
        
        max_value = max(values)
        max_index = values.index(max_value)

        direction = -1

        if max_value > threshold or max_value > threshold_dynamic: 
            if max_value > threshold and max_value < threshold_dynamic:
                self.get_logger().info(f'Max reading above static threshold at {max_index} is {max_value}')
            elif max_value > threshold_dynamic and max_value < threshold:
                self.get_logger().info(f'Max reading above dynamic threshold at {max_index} is {max_value}')
            else:
                self.get_logger().info(f'Max reading above both thresholds at {max_index} is {max_value}')
            

            if max_index == self.target_index:
                #self.stop()

                if self.distance_at_zero > threshold2:
                    self.forward()
                    self.get_logger().info('Moving forward')
                else:
                    self.stop()
                    self.align_complete = True
                    self.aligner_complete_pub.publish(Bool(data=self.align_complete))
                    self.align_now = False

            else:
                direction = self.get_rotation_direction(max_index)
                self.rotate(direction)
        else:
            if self.rotation_start_time is None:
                self.rotation_start_time = current_time
            if (current_time - self.rotation_start_time).nanoseconds / 1e9 <= self.rotation_timeout:
                self.rotate(direction)
            else:
                self.stop()
                self.align_complete = True
                self.aligner_complete_pub.publish(Bool(data=self.align_complete))
                self.align_now = False

    def align_service_callback(self, request, response):
        self.get_logger().info('Aligning robot...')
        self.align_now = True
        self.align_complete = False
        response.success = True
        response.message = 'Aligned robot...'
        return response

    def get_rotation_direction(self, max_index):
        if max_index < 4:
            return 1
        else:
            return -1

    def rotate(self, direction):
        twist = Twist()
        twist.angular.z = 0.3 * direction
        self.cmd_pub.publish(twist)

    def forward(self):
        twist = Twist()
        twist.linear.x = 0.05
        twist.angular.z = 0.0
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

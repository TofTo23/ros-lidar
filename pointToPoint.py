import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PointToPoint(Node):
    '''Realisation of point-to-point movement'''
    def __init__(self, x_target=1.0, y_target=1.0):
        super().__init__('point_to_point_node')
        
        self.x_target = x_target
        self.y_target = y_target
        self.near_target_position = 0.1
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_received = False # check if robot reach target point
        
        self.k_v = 0.5 
        self.k_w = 2.0

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.publishVel)
        
        self.get_logger().info(f"Target point ({self.x_target}, {self.y_target})")

    def odom_callback(self, msg):
        '''Getting current position'''
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)
        
        self.pose_received = True # flag of reaching target point

    def publishVel(self):
        '''Publish velocities'''
        if not self.pose_received:
            return
            
        dx = self.x_target - self.x
        dy = self.y_target - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        msg = Twist()
        
        if distance < self.near_target_position:
            self.get_logger().info("Arrived to target point")
            self.publisher.publish(msg)
            return
        
        # error of angle
        angle = math.atan2(dy, dx) - self.theta
        angle_norm = math.atan2(math.sin(angle), math.cos(angle))
        
        msg.angular.z = self.k_w * angle_norm

        # x is a direction of robot
        msg.linear.x = min(self.k_v * distance, 0.22) # limit of max speed
            
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    ptp = PointToPoint(x_target=2.0, y_target=5)
    
    try:
        rclpy.spin(ptp)
    except KeyboardInterrupt:
        ptp.get_logger().info("Program stopped")
    finally:
        stop_msg = Twist()
        ptp.publisher.publish(stop_msg)
        ptp.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
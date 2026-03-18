import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarReader(Node):
    def __init__(self):
        super().__init__('lidar_reader_node')


        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.get_logger().info("Dane z LIDARA: ")


    def lidar_callback(self, msg):        
        front = msg.ranges[0]
        left = msg.ranges[90]
        back = msg.ranges[180]
        right = msg.ranges[270]
        
        self.get_logger().info(
            f'Front: {front:.2f} Left: {left:.2f} Back: {back:.2f} Right: {right:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)

    lidar_node = LidarReader()
    
    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        lidar_node.get_logger().info("Stop")
    finally:
        lidar_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
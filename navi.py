import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import a_star
import calculateVff as cf
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# synthetic map
map_grid = np.array([[0, 1, 0, 0, 0, 0],
                     [0, 1, 0, 1, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [1, 1, 0, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0],
                     [0, 0, 1, 0, 0, 0]])

start_node = np.array([0, 0])
goal_node = np.array([5, 5])
rows, cols = map_grid.shape


class RobotControl(Node):
    '''Realisation of movement in a maze using A* and VFF'''
    def __init__(self, path, env_map):
        super().__init__('a_star_vff_node')

        self.init_x = None
        self.init_y = None
        
        self.path = path
        # store map for VFF
        self.env_map = env_map 
        self.near_target_position = 0.2

        # VFF parameters

        # area of interest for VFF
        self.vff_win_size = 5
        # adjust: 1.0 -> 1 m; 20.0 -> 5 cm etc.
        self.vff_scale = 1.0 

        self.pos_idx = 0
        self.x_target = float(self.path[self.pos_idx][0])
        self.y_target = float(self.path[self.pos_idx][1])

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pose_received = False 

        # containers for path
        self.trajectory_x = []
        self.trajectory_y = []

        # PI coefficients
        self.k_v = 0.5 
        self.k_s = 2.0

        self.planned_path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.actual_path_pub = self.create_publisher(Path, '/actual_path', 10)

        # actual path container
        self.actual_path_msg = Path()
        self.actual_path_msg.header.frame_id = 'odom'

        # planned path
        self.planned_path_msg = Path()
        self.planned_path_msg.header.frame_id = 'odom'
        
        for p in self.path:
            # for rviz
            pose = PoseStamped()
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            self.planned_path_msg.poses.append(pose)

        # publish velocities
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # subscribe odometry
        self.subscription = self.create_subscription(Odometry, '/odom', self.subscribeOdom, 10)
        self.timer = self.create_timer(0.1, self.publishVel)
        
        self.get_logger().info(f"Target point ({self.x_target}, {self.y_target})")

    def subscribeOdom(self, msg):
        '''Getting odometry for rviz'''

        # if first run of program
        if self.init_x is None:
            self.init_x = msg.pose.pose.position.x
            self.init_y = msg.pose.pose.position.y
            self.get_logger().info(f"Origin set at: {self.init_x}, {self.init_y}")

        self.x = msg.pose.pose.position.x - self.init_x
        self.y = msg.pose.pose.position.y - self.init_y

        self.trajectory_x.append(self.x)
        self.trajectory_y.append(self.y)
        
        # angle coords
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.theta = 2.0 * math.atan2(qz, qw)
        
        self.pose_received = True 

        current_time = self.get_clock().now().to_msg()

        # get odom from robot
        current_pose = PoseStamped()
        current_pose.header.stamp = current_time
        current_pose.header.frame_id = 'odom'
        current_pose.pose.position.x = self.x
        current_pose.pose.position.y = self.y
        current_pose.pose.orientation = msg.pose.pose.orientation
        
        # update actual path
        self.actual_path_msg.poses.append(current_pose)
        
        # publish paths
        self.actual_path_msg.header.stamp = current_time
        self.planned_path_msg.header.stamp = current_time
        
        self.actual_path_pub.publish(self.actual_path_msg)
        self.planned_path_pub.publish(self.planned_path_msg)

    def publishVel(self):
        if not self.pose_received:
            return
        
        dx = self.x_target - self.x
        dy = self.y_target - self.y
        distance = math.sqrt(dx**2 + dy**2)
        
        msg = Twist()
        
        # contition for reaching current node of A* algorithm
        if distance < self.near_target_position:
            self.pos_idx += 1
            if self.pos_idx >= len(self.path):
                self.get_logger().info("Arrived to goal point")
                self.publisher.publish(msg)
                raise SystemExit
            else:
                self.x_target = float(self.path[self.pos_idx][0])
                self.y_target = float(self.path[self.pos_idx][1])
                self.get_logger().info(f"Next target point: ({self.x_target}, {self.y_target})")
                return
        
        # VFF calculations
        pose = np.array([self.x, self.y])
        target = np.array([self.x_target, self.y_target])
        
        Fa, Fr = cf.calculateVff(pose, target, self.env_map, self.vff_win_size, self.vff_scale)
        
        # resultant force
        F_total = Fa + Fr
        
        # prevent local minimus points
        if np.linalg.norm(F_total) < 1e-6:
            F_total = np.array([math.cos(self.theta), math.sin(self.theta)])

        # use VFF angle to direct
        desired_angle = math.atan2(F_total[1], F_total[0])
        
        angle_error = desired_angle - self.theta
        angle_norm = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        msg.angular.z = self.k_s * angle_norm

        
        # max linear speed TODO: adjust to the real robot velocity
        msg.linear.x = min(self.k_v * distance, 0.22) 
            
        self.publisher.publish(msg)

def main(args=None):
    # global path is obtained by A*
    path = a_star.a_star(map_grid, start_node, goal_node)

    rclpy.init(args=args)

    robot1 = RobotControl(path, map_grid)

    try:
        rclpy.spin(robot1)
    except KeyboardInterrupt:
        robot1.get_logger().info("Program stopped")
    finally:
        stop_msg = Twist()
        robot1.publisher.publish(stop_msg)
        
        # extract xy coords from the A* path
        path_x = [p[0] for p in path]
        path_y = [p[1] for p in path]

  
        plt.figure(figsize=(8, 8))
        plt.imshow(map_grid, cmap='Greys', origin='lower')
        plt.plot(path_y, path_x, 'r--', label='Planned path', linewidth=2) # red -> planned path (A*)
        plt.plot(robot1.trajectory_y, robot1.trajectory_x, 'b-', label='Actual trajectory', linewidth=2) # blue -> real trajectory (VFF)
        
        plt.title('Robot navigation')
        plt.xlabel('y')
        plt.ylabel('x')
        plt.legend()
        plt.grid(True)
        
        plt.show()

        # clean up ROS 2
        robot1.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
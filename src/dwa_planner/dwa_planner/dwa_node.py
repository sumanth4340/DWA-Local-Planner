import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import numpy as np
import math

class DWANavigationNode(Node):
    def __init__(self):
        super().__init__('dwa_planner_node')
        self.declare_parameter('max_speed', 0.22)
        self.MAX_V = self.get_parameter('max_speed').value
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/dwa_trajectory', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.pose = None
        self.yaw = 0.0
        self.scan = None
        self.goal = None
        self.MIN_V = 0.0
        self.MAX_W = 2.84
        self.DT = 0.1
        self.PREDICT_TIME = 2.0
        self.get_logger().info("DWA Planner initialized. Set a goal in RViz to start.")

    def goal_callback(self, msg):
        self.goal = np.array([msg.pose.position.x, msg.pose.position.y])
        self.get_logger().info(f"New Goal Received: x={self.goal[0]:.2f}, y={self.goal[1]:.2f}")

    def odom_callback(self, msg):
        self.pose = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

    def scan_callback(self, msg):
        self.scan = [r for r in msg.ranges if 0.12 < r < msg.range_max]

    def control_loop(self):
        if self.pose is None or self.scan is None or self.goal is None:
            return
        curr_pos = np.array([self.pose.x, self.pose.y])
        dist_to_goal = np.linalg.norm(curr_pos - self.goal)
        
        if dist_to_goal < 0.15:
            self.stop_robot()
            return
        best_v, best_w, best_traj = self.dwa_planner()
        
        self.publish_vel(best_v, best_w)
        if best_traj is not None:
            self.publish_trajectory_marker(best_traj)

    def dwa_planner(self):
        best_score = -float('inf')
        best_v, best_w, best_traj = 0.0, 0.0, None
        for v in np.linspace(self.MIN_V, self.MAX_V, 6):
            for w in np.linspace(-self.MAX_W, self.MAX_W, 11):
                traj = self.predict_trajectory(v, w)
                score = self.evaluate_trajectory(traj, v, w)
                
                if score > best_score:
                    best_score = score
                    best_v = v
                    best_w = w
                    best_traj = traj
        
        return best_v, best_w, best_traj

    def predict_trajectory(self, v, w):
        x, y, yaw = self.pose.x, self.pose.y, self.yaw
        trajectory = []
        for _ in range(int(self.PREDICT_TIME / self.DT)):
            x += v * math.cos(yaw) * self.DT
            y += v * math.sin(yaw) * self.DT
            yaw += w * self.DT
            trajectory.append([x, y])
        return np.array(trajectory)

    def evaluate_trajectory(self, trajectory, v, w):
        goal_cost = -np.linalg.norm(trajectory[-1] - self.goal)
        min_dist = min(self.scan) if self.scan else 0.0
        obstacle_cost = min_dist
        speed_cost = v 
        return (2.5 * goal_cost) + (4.0 * obstacle_cost) + (1.2 * speed_cost)

    def publish_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_pub.publish(msg)

    def stop_robot(self):
        self.publish_vel(0.0, 0.0)
        self.get_logger().info("Goal Reached! Stopping.")
        self.goal = None 

    def publish_trajectory_marker(self, trajectory):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "dwa_planner"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.03
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.0, 1.0, 0.0, 1.0
        
        for pt in trajectory:
            marker.points.append(Point(x=pt[0], y=pt[1], z=0.01))
        
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = DWANavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

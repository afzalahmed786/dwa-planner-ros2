#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy


class DWAConfig:
    """Configuration parameters for DWA planner"""
    def __init__(self):
        # Robot specifications (more conservative for TurtleBot3)
        self.max_speed = 0.15  # m/s (reduced from 0.22)
        self.min_speed = -0.15  # m/s (reduced from -0.22)
        self.max_yaw_rate = 1.5  # rad/s (reduced from 2.84)
        self.max_accel = 0.5  # m/s^2 (reduced from 2.5)
        self.max_delta_yaw_rate = 1.0  # rad/s^2 (reduced from 3.2)
        
        # Velocity resolution
        self.v_resolution = 0.02  # m/s (increased for fewer samples)
        self.yaw_rate_resolution = 0.2  # rad/s (increased for fewer samples)
        
        # Simulation parameters
        self.dt = 0.1  # time step (s)
        self.predict_time = 2.0  # prediction horizon (s) (reduced from 3.0)
        
        # Cost function weights
        self.heading_cost_gain = 1.0  # Increased to prioritize heading
        self.distance_cost_gain = 1.0
        self.velocity_cost_gain = 0.5  # Reduced to allow slower movement
        self.obstacle_cost_gain = 2.0  # Increased for safety
        
        # Robot radius for collision checking
        self.robot_radius = 0.22  # m
        
        # Goal tolerance
        self.goal_tolerance = 0.3  # m

class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')
        
        # Configuration
        self.config = DWAConfig()
        
        # State variables
        self.current_pose = None
        self.current_velocity = np.array([0.0, 0.0])  # [v, omega]
        self.laser_data = None
        self.goal_pose = None
        
        # QoS profile
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos)
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dwa_trajectories', 10)
        
        # Timer for control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('DWA Planner initialized')
    
    def odom_callback(self, msg):
        """Extract current pose and velocity from odometry"""
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.quaternion_to_yaw(msg.pose.pose.orientation)
        ])
        
        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        ])
    
    def scan_callback(self, msg):
        """Store laser scan data"""
        self.laser_data = msg
    
    def goal_callback(self, msg):
        """Set new goal pose"""
        self.goal_pose = np.array([
            msg.pose.position.x,
            msg.pose.position.y
        ])
        self.get_logger().info(f'New goal received: ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f})')
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def dynamic_window(self):
        """Calculate the dynamic window based on robot constraints"""
        # Velocity bounds from robot constraints
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]
        
        # Dynamic window from current velocity
        Vd = [
            self.current_velocity[0] - self.config.max_accel * self.config.dt,
            self.current_velocity[0] + self.config.max_accel * self.config.dt,
            self.current_velocity[1] - self.config.max_delta_yaw_rate * self.config.dt,
            self.current_velocity[1] + self.config.max_delta_yaw_rate * self.config.dt
        ]
        
        # Intersection of velocity space and dynamic window
        dw = [
            max(Vs[0], Vd[0]),
            min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]),
            min(Vs[3], Vd[3])
        ]
        
        return dw
    
    def predict_trajectory(self, v, omega):
        """Predict robot trajectory for given velocity"""
        x, y, theta = self.current_pose
        trajectory = [[x, y, theta]]
        time = 0
        
        while time <= self.config.predict_time:
            x += v * math.cos(theta) * self.config.dt
            y += v * math.sin(theta) * self.config.dt
            theta += omega * self.config.dt
            trajectory.append([x, y, theta])
            time += self.config.dt
        
        return np.array(trajectory)
    
    def calc_obstacle_cost(self, trajectory):
        """Calculate cost based on obstacle proximity"""
        if self.laser_data is None:
            return 0.0
        
        min_dist = float('inf')
        
        # Convert laser scan to points
        angle = self.laser_data.angle_min
        for r in self.laser_data.ranges:
            if r < self.laser_data.range_min or r > self.laser_data.range_max:
                angle += self.laser_data.angle_increment
                continue
            
            # Obstacle position in robot frame
            ox = r * math.cos(angle)
            oy = r * math.sin(angle)
            
            # Check distance to trajectory points
            for point in trajectory:
                # Transform obstacle to map frame
                obs_x = self.current_pose[0] + ox * math.cos(self.current_pose[2]) - oy * math.sin(self.current_pose[2])
                obs_y = self.current_pose[1] + ox * math.sin(self.current_pose[2]) + oy * math.cos(self.current_pose[2])
                
                dx = point[0] - obs_x
                dy = point[1] - obs_y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist < min_dist:
                    min_dist = dist
            
            angle += self.laser_data.angle_increment
        
        # Cost is inversely proportional to distance
        if min_dist < self.config.robot_radius:
            return float('inf')  # Collision
        
        return 1.0 / min_dist
    
    def calc_to_goal_cost(self, trajectory):
        """Calculate cost based on distance to goal"""
        if self.goal_pose is None:
            return 0.0
        
        dx = self.goal_pose[0] - trajectory[-1, 0]
        dy = self.goal_pose[1] - trajectory[-1, 1]
        goal_dist = math.sqrt(dx**2 + dy**2)
        
        return goal_dist
    
    def calc_heading_cost(self, trajectory):
        """Calculate cost based on heading to goal"""
        if self.goal_pose is None:
            return 0.0
        
        dx = self.goal_pose[0] - trajectory[-1, 0]
        dy = self.goal_pose[1] - trajectory[-1, 1]
        goal_heading = math.atan2(dy, dx)
        
        heading_diff = abs(goal_heading - trajectory[-1, 2])
        heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
        
        return abs(heading_diff)
    
    def calc_velocity_cost(self, v):
        """Encourage higher velocities"""
        return self.config.max_speed - v
    
    def dwa_control(self):
        """Main DWA planning algorithm"""
        dw = self.dynamic_window()
        
        best_cost = float('inf')
        best_v = 0.0
        best_omega = 0.0
        best_trajectory = None
        
        trajectories = []
        
        # Sample velocities within dynamic window
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for omega in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):
                # Predict trajectory
                trajectory = self.predict_trajectory(v, omega)
                
                # Calculate costs
                heading_cost = self.calc_heading_cost(trajectory)
                dist_cost = self.calc_to_goal_cost(trajectory)
                vel_cost = self.calc_velocity_cost(v)
                obs_cost = self.calc_obstacle_cost(trajectory)
                
                # Total cost
                total_cost = (
                    self.config.heading_cost_gain * heading_cost +
                    self.config.distance_cost_gain * dist_cost +
                    self.config.velocity_cost_gain * vel_cost +
                    self.config.obstacle_cost_gain * obs_cost
                )
                
                trajectories.append((trajectory, total_cost))
                
                # Update best trajectory
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_v = v
                    best_omega = omega
                    best_trajectory = trajectory
        
        # Visualize trajectories
        self.visualize_trajectories(trajectories, best_trajectory)
        
        return best_v, best_omega, best_trajectory
    

    def visualize_trajectories(self, trajectories, best_trajectory):
        """Publish trajectory markers for RViz"""
        marker_array = MarkerArray()
        
        # Visualize sampled trajectories
        for i, (traj, cost) in enumerate(trajectories[:50]):  # Limit to 50 for performance
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "trajectories"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01
            marker.color.a = 0.3
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            
            for point in traj:
                p = Point()  # Changed from geometry_msgs.msg.Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        # Visualize best trajectory
        if best_trajectory is not None:
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "best_trajectory"
            marker.id = 1000
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            
            for point in best_trajectory:
                p = Point()  # Changed from geometry_msgs.msg.Point()
                p.x = float(point[0])
                p.y = float(point[1])
                p.z = 0.0
                marker.points.append(p)
            
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)
    

    def control_loop(self):
        """Main control loop"""
        # Debug logging - check what data we have
        self.get_logger().info(
            f'Status - Pose: {self.current_pose is not None}, '
            f'Laser: {self.laser_data is not None}, '
            f'Goal: {self.goal_pose is not None}'
        )

        if self.current_pose is None:
            self.get_logger().warn('Waiting for odometry data...')
            return

        if self.laser_data is None:
            self.get_logger().warn('Waiting for laser scan data...')
            return

        if self.goal_pose is None:
            self.get_logger().info('No goal set yet. Waiting for goal pose...')
            return

        # Check if goal is reached
        dx = self.goal_pose[0] - self.current_pose[0]
        dy = self.goal_pose[1] - self.current_pose[1]
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        self.get_logger().info(
            f'Current: ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}) | '
            f'Goal: ({self.goal_pose[0]:.2f}, {self.goal_pose[1]:.2f}) | '
            f'Distance: {dist_to_goal:.2f}m'
        )

        if dist_to_goal < self.config.goal_tolerance:
            self.get_logger().info('Goal reached!')
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            self.goal_pose = None
            return

        # Run DWA planner
        v, omega, trajectory = self.dwa_control()

        # Publish velocity command
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = omega
        self.cmd_vel_pub.publish(cmd_vel)

        # Log debug information
        self.get_logger().info(
            f'Publishing cmd_vel: v={v:.3f} m/s, ω={omega:.3f} rad/s'
        )

def main(args=None):
    rclpy.init(args=args)
    planner = DWAPlanner()
    
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

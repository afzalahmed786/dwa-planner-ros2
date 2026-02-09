#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Declare parameters
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        
        # Publish goal after a short delay
        self.timer = self.create_timer(2.0, self.publish_goal)
        self.published = False
    
    def publish_goal(self):
        if not self.published:
            goal = PoseStamped()
            goal.header.frame_id = 'odom'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = self.get_parameter('goal_x').value
            goal.pose.position.y = self.get_parameter('goal_y').value
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            
            self.publisher.publish(goal)
            self.get_logger().info(
                f'Published goal: ({goal.pose.position.x}, {goal.pose.position.y})'
            )
            self.published = True

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

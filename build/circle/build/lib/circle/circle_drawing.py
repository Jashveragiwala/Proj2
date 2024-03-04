#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
import time
import math

class RVizStraightLinePublisher(Node):
    def __init__(self):
        super().__init__('rviz_straight_line_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # Wait for some time to make sure the initial pose publisher is ready
        time.sleep(1)
        # self.timer = self.create_timer(0.1, self.publish_twist_message)  # Use a smaller timer interval
        # self.move_start_time = time.time()
        # self.turn_start_time = None
    
    def set_initial_pose(self, x=1.0, y=1.0):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.position.z = 0.0  # Assuming 2D pose

        self.get_logger().info(f"Setting initial pose to ({x}, {y})")
        self.initial_pose_publisher.publish(initial_pose_msg)

    def publish_twist_message(self,duration=5.0, linear_speed=0.2, rotate_duration=1.0, angular_speed=math.pi/2):
        twist_msg = Twist()
        twist_msg.linear.x = linear_speed

        self.get_logger().info("Moving forward...")
        start_time = time.time()
        while time.time() - start_time < duration:
            self.publisher.publish(twist_msg)
            self.get_logger().info("Publishing Twist command")
            time.sleep(0.1)

        

        # Stop the robot after the specified duration
        twist_msg.linear.x = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info("Stopped.")

        # Rotate in place
        twist_msg.angular.z = angular_speed
        self.get_logger().info("Rotating in place...")
        start_time = time.time()
        while time.time() - start_time < rotate_duration:
            self.publisher.publish(twist_msg)
            self.get_logger().info("Publishing Twist command")
            time.sleep(0.1)

        # Stop the rotation after the specified rotation duration
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info("Rotation complete.")



def main(args=None):
    rclpy.init(args=args)
    rviz_straight_line_publisher = RVizStraightLinePublisher()
    rviz_straight_line_publisher.set_initial_pose()
    rviz_straight_line_publisher.publish_twist_message()
    # rclpy.spin(rviz_straight_line_publisher)
    rviz_straight_line_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

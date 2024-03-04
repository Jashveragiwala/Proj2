#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class RVizStraightLinePublisher(Node):
    def __init__(self):
        super().__init__('rviz_straight_line_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_twist_message)

    def publish_twist_message(self):
        twist_msg = Twist()
        twist_msg.linear.x = -0.2  # Set the linear velocity for a straight line

        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    rviz_straight_line_publisher = RVizStraightLinePublisher()
    rclpy.spin(rviz_straight_line_publisher)
    rviz_straight_line_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

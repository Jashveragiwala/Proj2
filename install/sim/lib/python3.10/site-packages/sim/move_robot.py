#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobot(Node):
    def __init__(self):
        super().__init__('move_robot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    def move(self):
        twist_msg = Twist()
        twist_msg.linear.x = 1.0  # Move forward
        self.publisher.publish(twist_msg)
        time.sleep(2)  # Move for 2 seconds

def main():
    rclpy.init()
    move_robot_node = MoveRobot()

    # Move the robot
    move_robot_node.move()

    move_robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

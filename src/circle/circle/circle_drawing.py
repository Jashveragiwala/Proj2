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
        # self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        # # Wait for some time to make sure the initial pose publisher is ready
        # time.sleep(1)
        # self.timer = self.create_timer(0.1, self.publish_twist_message)  # Use a smaller timer interval
        # self.move_start_time = time.time()
        # self.turn_start_time = None
    
    # def set_initial_pose(self, x=1.0, y=1.0):
    #     initial_pose_msg = PoseWithCovarianceStamped()
    #     initial_pose_msg.pose.pose.position.x = x
    #     initial_pose_msg.pose.pose.position.y = y
    #     initial_pose_msg.pose.pose.position.z = 0.0  # Assuming 2D pose

    #     self.get_logger().info(f"Setting initial pose to ({x}, {y})")
    #     self.initial_pose_publisher.publish(initial_pose_msg)

    def publish_twist_message(self,duration,rotate_duration,angular_speed,linear_speed):
        twist_msg = Twist()
        

        for i in range(0,len(duration)-1):
            # Rotate in place
            
            if rotate_duration[i]<0:
                twist_msg.angular.z = -angular_speed
                rot_time = abs(rotate_duration[i])

            else:
                twist_msg.angular.z = angular_speed
                rot_time = rotate_duration[i]
            self.get_logger().info("Rotating in place...")
            start_time = time.time()
            while time.time() - start_time < rot_time:
                self.publisher.publish(twist_msg)
                self.get_logger().info("Publishing Twist command")
                time.sleep(0.1)

            # Stop the rotation after the specified rotation duration
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)
            self.get_logger().info("Rotation complete.")


            twist_msg.linear.x = linear_speed

            self.get_logger().info("Moving forward...")
            start_time = time.time()
            while time.time() - start_time < duration[i]:
                self.publisher.publish(twist_msg)
                self.get_logger().info("Publishing Twist command")
                time.sleep(0.1)

            # Stop the robot after the specified duration
            twist_msg.linear.x = 0.0
            self.publisher.publish(twist_msg)
            self.get_logger().info("Stopped.")

        



def main(args=None):
    rclpy.init(args=args)
    rviz_straight_line_publisher = RVizStraightLinePublisher()
    coord = [(0, 0), (-3.73, 2.64), (3.75, 2.64), (3.75, 0.54), (2.327, 0.54), (2.327, 1.36), (0.73,1.36),]
    distances = []
    speed = 1.0
    angular_speed = 0.1
    time = []
    angles = []
    angular_times = []
    angle_rad_list =[]

    for i in range(len(coord) - 1):
        x1, y1 = coord[i]
        x2, y2 = coord[i + 1]

        # Calculate distance
        distance = round(math.sqrt((x2 - x1)**2 + (y2 - y1)**2), 4)
        distances.append(distance)

        # Calculate time
        time.append(round(distance / speed, 4))

        # Calculate angle with x-axis
        angle_rad = math.atan2(y2 - y1, x2 - x1)
        angle_deg = round(math.degrees(angle_rad),4)
        if i == 0:
            angle_deg -= 0
        else:
            angle_deg -= round(sum(angles[:i]),4) 
        angles.append(angle_deg)

        # Convert angle_deg back to radians for angular time calculation
        angle_rad = math.radians(angle_deg)
        angle_rad_list.append(angle_rad)
        # Calculate angular time
        angular_time = angle_rad / angular_speed
        angular_times.append(round(angular_time, 4))


    print("Distances between consecutive points:")
    print(distances)

    print("Time taken for each segment:")
    print(time)

    print("Angles between each line segment and x-axis (in degrees):")
    print(angles)

    print("Angular time for each segment:")
    print(angular_times)
    
    # rviz_straight_line_publisher.set_initial_pose()
    rviz_straight_line_publisher.publish_twist_message(time, angular_times, 0.1, 1.0)

    

if __name__ == '__main__':
    main()

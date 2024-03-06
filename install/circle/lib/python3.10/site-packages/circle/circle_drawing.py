#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist, Pose2D
from std_srvs.srv import SetBool
import time
import math



class RVizStraightLinePublisher(Node):
    def __init__(self):
        super().__init__('rviz_straight_line_publisher')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.set_pen_client = self.create_client(SetBool, '/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /set_pen not available, waiting again...')
        self.timer = self.create_timer(0.1, self.publish_twist_message)
        self.current_pose = None
        # self.pose_subscription = self.create_subscription(
        #     Pose2D,
        #     '/pose',  # Replace with the actual topic for the robot's pose
        #     self.pose_callback,
        #     10)
        # self.current_pose = None
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
    def set_pen(self, value):
        req = SetBool.Request()
        req.data = value
        future = self.set_pen_client.call_async(req)

    def publish_twist_message(self,duration,rotate_duration,angular_speed,linear_speed):
        twist_msg = Twist()
        

        for i in range(0,len(duration)-1):
            # Rotate in place
            if i==0 or i==17 or i==38 or i==42 or i==67:
                self.set_pen(False)
            
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
                time.sleep(0.1)
            self.get_logger().info("Publishing Twist command")

            # Stop the rotation after the specified rotation duration
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)
            self.get_logger().info("Rotation complete.")


            twist_msg.linear.x = linear_speed

            self.get_logger().info("Moving forward...")
            start_time = time.time()
            while time.time() - start_time < duration[i]:
                self.publisher.publish(twist_msg)
                time.sleep(0.1)
            self.get_logger().info("Publishing Twist command")

            # Stop the robot after the specified duration
            twist_msg.linear.x = 0.0
            self.publisher.publish(twist_msg)
            self.get_logger().info("Stopped.")

            # self.get_logger().info(f"Current Position: ({self.current_pose.x}, {self.current_pose.y})")

            if (i==0 or i==17 or i==38 or i==42 or i==67):
                self.set_pen(True)

    def pose_callback(self, msg):
        # Update the current pose whenever a new pose message is received
        self.current_pose = msg.pose

        
def main(args=None):
    rclpy.init(args=args)
    rviz_straight_line_publisher = RVizStraightLinePublisher()
    coord = [(0, 0), (-3.73, 2.64), (3.75, 2.64), (3.75, 0.54), (2.327, 0.54), (2.327, 1.36), (0.73,1.36),(0.73,-3.54),(1.53,-3.54),(1.53,-5),(-1.66,-5),(-1.66,-3.54),(-0.705,-3.54),(-0.705,1.36),(-2.296,1.36),(-2.296,0.54),(-3.73,0.54),(-3.73,2.64),
            (-3.73,0.0885),(-3.73,-0.532),(-3.568,-0.532),(-4.39,-2.43),(-4.866,-2.43),(-4.866,-3.22),(-3.417,-3.22),(-3.417,-2.43),(-3.57,-2.43),(-3.394,-2.126),(-2.626,-2.126),(-2.46,-2.43),(-2.6,-2.43),(-2.6,-3.22),(-1.185,-3.22),(-1.185,-2.43),(-1.49,-2.43),(-2.456,-0.532),(-2.296,-0.532),(-2.296,0.0886),(-3.73,0.086),(-2.997,-1.067),(-2.779,-1.479),(-3.224,-1.475),(-2.997,-1.067),
            (1.21,0.0885),(2.32,0.0885),(2.96,-1.313),(3.75,0.0885),(4.866,0.0885),(4.866,-0.532),(4.57,-0.532),(4.57,-2.43),(4.866,-2.43),(4.866,-3.22),(3.6,-3.22),(3.6,-2.43),(3.91,-2.43),(3.91,-1.154),(3.12,-2.74),(2.15,-1.154),(2.15,-2.43),(2.476,-2.43),(2.476,-3.22),(1.21,-3.22),(1.21,-2.43),(1.51,-2.43),(1.51,-0.532),(1.21,-0.532),(1.21,0.0885),(5,-5)]


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
    rviz_straight_line_publisher.publish_twist_message(time, angular_times, angular_speed, speed)

    

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist,Pose2D
# import numpy as np

# FORWARD_VELOCITY = 1
# ROTATIONAL_VELOCITY = 2
# LEFT_VELOCITY = 0.1

# coord = [(0,0),(-3.73, 2.64),(3.75, 2.64), (3.75, 0.54), (2.327, 0.54), (2.327, 1.36), (0.73,1.36),(0.73,-3.54),(1.53,-3.54),(1.53,-5),(-1.66,-5),(-1.66,-3.54),(-0.705,-3.54),(-0.705,1.36),(-2.296,1.36),(-2.296,0.54),(-3.73,0.54),(-3.73,2.64),
#             (-3.73,0.0885),(-3.73,-0.532),(-3.568,-0.532),(-4.39,-2.43),(-4.866,-2.43),(-4.866,-3.22),(-3.417,-3.22),(-3.417,-2.43),(-3.57,-2.43),(-3.394,-2.126),(-2.626,-2.126),(-2.46,-2.43),(-2.6,-2.43),(-2.6,-3.22),(-1.185,-3.22),(-1.185,-2.43),(-1.49,-2.43),(-2.456,-0.532),(-2.296,-0.532),(-2.296,0.0886),(-3.73,0.086),(-2.997,-1.067),(-2.779,-1.479),(-3.224,-1.475),(-2.997,-1.067),
#             (1.21,0.0885),(2.32,0.0885),(2.96,-1.313),(3.75,0.0885),(4.866,0.0885),(4.866,-0.532),(4.57,-0.532),(4.57,-2.43),(4.866,-2.43),(4.866,-3.22),(3.6,-3.22),(3.6,-2.43),(3.91,-2.43),(3.91,-1.154),(3.12,-2.74),(2.15,-1.154),(2.15,-2.43),(2.476,-2.43),(2.476,-3.22),(1.21,-3.22),(1.21,-2.43),(1.51,-2.43),(1.51,-0.532),(1.21,-0.532),(1.21,0.0885)]


# class MoveRobot(Node):
#     def __init__(self):
#         super().__init__("move_robot")
#         self.get_logger().info(f"1")
#         self.publisher_ = self.create_publisher(Twist,"/cmd_vel",10)
#         self.get_logger().info(f"2")
#         self.timer_ = self.create_timer(10,self.publish_news)
#         self.get_logger().info(f"3")
#         self.linear = 0
#         self.angular = 0
#         self.curr_x_set_point = 0
#         self.curr_y_set_point = 0
#         self.curr_theta_set_point = 0
#         self.rotation_done = False
#         self.dest_x = 0  # Initialize dest_x
#         self.dest_y = 0 
        
#         # self.linear_motion = False

#         # self.subscriber_ = self.create_subscription("Pose2D","/pose",self.callback,10)
#         # self.subscriber_ = self.create_subscription(Pose2D,"/pose",self.callback,10)
#         self.subscription = self.create_subscription(
#             Pose2D,
#             '/pose',  # Replace with the actual topic name
#             self.callback,
#             10  # QoS profile depth
#         )
#         # self.subscriber_ = self.create_subscription(Pose2D, "/pose", self.callback, 10)
#         self.get_logger().info(f"5")
#         self.rotation_done = False
       
#     def callback(self,msg):
#         # self.get_logger().info(f"blue:{msg.b},red:{msg.r},green:{msg.g}")
#         self.get_logger().info(f"6")
#         curr_x = msg.x
#         curr_y = msg.y
#         curr_theta = msg.theta
#         self.get_logger().info(f"4")
#         self.get_logger().info(f"Current Position: ({curr_x}, {curr_y},{curr_theta})")
        
#         self.curr_x_set_point = curr_x
#         self.curr_y_set_point = curr_y
#         self.curr_theta_set_point = curr_theta

#         if not self.rotation_done:
#             # Calculate the angle difference
#             angle_diff = self.dest_theta - curr_theta

#             # Adjust angular velocity for rotation
#             self.angular = np.clip(LEFT_VELOCITY * angle_diff, -ROTATIONAL_VELOCITY, ROTATIONAL_VELOCITY)

#             # Check if rotation is done
#             if abs(angle_diff) < 1.0:
#                 self.rotation_done = True
#                 self.angular = 0
#         else:
#             # Calculate the distance and angle to the destination
#             distance = np.linalg.norm(np.array([curr_x - self.dest_x, curr_y - self.dest_y]))
#             angle_rad = np.arctan2(self.dest_y - curr_y, self.dest_x - curr_x)

#             # Adjust linear velocity for translation
#             self.linear = min(FORWARD_VELOCITY, distance)

#         # if(self.linear_motion):
#         #     self.linear = FORWARD_VELOCITY*(np.linalg.norm(np.array([self.curr_x_set_point-curr_x,self.curr_y_set_point-curr_y])))
#         #     if(self.linear<10e-5): 
#         #         self.linear_motion=False

#         # else:
#         #     self.angular = LEFT_VELOCITY*(np.abs(self.curr_theta_set_point-curr_theta))
#         #     if(self.angular<10e-5): 
#         #         self.linear_motion=True
        
        
#     def done(self):
#         return self.linear < 10e-5 

#     def publish_news(self):
#         msg = Twist()
#         msg.linear.x= float(self.linear)
#         msg.angular.z= float(self.angular)
#         self.publisher_.publish(msg)
        

# def main(args=None):
#     rclpy.init(args=args)
#     node = MoveRobot()

#     for dest_x, dest_y in coord:
#         print(dest_x, dest_y)
#         # rclpy.spin_once(node)  # Spin once to process the callback and update current position
#         print(f"Current Position2: ({node.curr_x_set_point}, {node.curr_y_set_point}, {node.curr_theta_set_point})")

#         node.dest_x = dest_x
#         node.dest_y = dest_y
#         node.rotation_done = False

#         while not node.done():
#             rclpy.spin_once(node)

#         print(f"Reached Destination: ({node.curr_x_set_point}, {node.curr_y_set_point}, {node.curr_theta_set_point})")

#         # Calculate the destination angle
#         # if (dest_x - node.curr_x_set_point) == 0:
#         #     angle_rad = np.sign(dest_y - node.curr_y_set_point) * np.pi / 2
#         # else:
#         #     angle_rad = np.arctan2(dest_y - node.curr_y_set_point, dest_x - node.curr_x_set_point)

#         # angle_deg = np.degrees(angle_rad)
#         # print(f"Desired Angle: {angle_deg} degrees")

        
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


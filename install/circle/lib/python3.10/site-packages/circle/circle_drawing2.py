#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CentroidSubscriber(Node):
    def __init__(self):
        super().__init__('centroid_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'centroid_topic',
            self.centroid_callback,
            10)

    def centroid_callback(self, msg):
        centroids = []
        for i in range(0, len(msg.data), 2):
            centroids.append((msg.data[i], msg.data[i+1]))
        print("Received centroids:", centroids)

def main(args=None):
    rclpy.init(args=args)
    centroid_subscriber = CentroidSubscriber()
    rclpy.spin(centroid_subscriber)

    centroid_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

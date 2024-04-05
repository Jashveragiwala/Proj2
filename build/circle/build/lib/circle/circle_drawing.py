#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
from sklearn.cluster import DBSCAN



class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        # self.publisher_ = self.create_publisher(MarkerArray, 'person_markers', 10)
        self.publisher_ = self.create_publisher(Float64MultiArray, 'centroid_topic', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.scan_data = []
        self.prev = None
        self.curr = None
        self.count=0

    def scan_callback(self, msg):
        self.scan_data.append(msg.ranges)
        ranges = msg.ranges
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        detected_people = self.detect_people(ranges,angle_min,angle_increment)
        # print(detected_people)
    
    def detect_people(self, ranges, angle_min,angle_increment):
        # Convert polar coordinates to Cartesian
        ranges_cart= self.polar_to_cartesian(ranges, angle_min, angle_increment)
        if self.count==0:
            self.curr=ranges_cart
        # Apply any required preprocessing (e.g., filtering)
        else:
            self.prev = self.curr 
            self.curr = ranges_cart
        self.count +=1

        if (self.count>=2) and (self.count<50):
            # print("previous",self.prev)
            # print("curr", self.curr)
            result = []
            for tuple_a, tuple_b in zip(self.prev, self.curr):
                temp = []
                for x, y in zip(tuple_a, tuple_b):
                    try:
                        # Convert strings to float and subtract
                        if(x=='inf' or x=="-inf"): x = 0
                        if(y=='inf' or y=="-inf"): y = 0
                        sub = float(x) - float(y)
                        # Check if result is inf, if so replace with 0
                        if sub == float('inf') or sub == float('-inf'):
                            sub = 0
                        # Round to three decimal places
                        sub = round(sub, 5)
                    except ValueError:
                        # Handle case where one of the values cannot be converted to float (e.g., 'nan')
                        sub = 0
                    temp.append(sub)
                result.append(temp)

            print("")
            diff =[]
            values = []

            difference = np.array(result)
            for elem in range(len(difference)):
                value = np.linalg.norm(difference[elem])
                if(value>0.5): 
                    diff.append("M")
                    values.append(value)
                else: 
                    diff.append("S")
                    values.append(value)
            # print(diff)

            m_indices = [values[i] for i, x in enumerate(diff) if x == 'M']
            m_index = [i for i, x in enumerate(diff) if x == 'M']
            result_dict = {m_indices[i]: m_index[i] for i in range(len(m_indices))}
            print(m_index)
            # Clustering using DBSCAN
            if m_indices:
                X = np.array(m_indices).reshape(-1, 1)
                dbscan = DBSCAN(min_samples=4, eps=0.25).fit(X)

                # Getting cluster labels
                labels = dbscan.labels_

                # Getting cluster indices
                cluster_indices = {}
                for label, index in zip(labels, m_indices):
                    if label != -1:
                        if label not in cluster_indices:
                            cluster_indices[label] = []
                        cluster_indices[label].append(index)

                # Filter clusters with more than 4 consecutive 'M' values
                valid_clusters = {label: indices for label, indices in cluster_indices.items() if len(indices) >= 4}

                print("Valid clusters:", valid_clusters)
                print()
                print(result_dict)
                # Assuming result_dict is already defined
                
                valid_clusters_key=list(valid_clusters.keys())
                valid_clusters_values=list(valid_clusters.values())

                valid_ind ={}
                for i in range(len(valid_clusters_key)):
                    l = []
                    for j in valid_clusters_values[i]:
                        
                        l.append(self.prev[result_dict[j]])
                    valid_ind[valid_clusters_key[i]]=l

                print(valid_ind)

                # Assuming valid_ind is already defined
                centroid_dict = {}

                for label, coordinates in valid_ind.items():
                    x_sum = 0
                    y_sum = 0
                    num_points = len(coordinates)
                    
                    for coord in coordinates:
                        x_sum += float(coord[0])
                        y_sum += float(coord[1])
                    
                    centroid_x = x_sum / num_points
                    centroid_y = y_sum / num_points
                    
                    centroid_dict[label] = (centroid_x, centroid_y)

                print("Centroids:", centroid_dict)
                print("Centroids list",list(centroid_dict.values()))

                centroid_msg = Float64MultiArray()
                centroid_msg.data = [coord for centroid in centroid_dict.values() for coord in centroid]
                self.publisher_.publish(centroid_msg)
            else:
                print("No points found for clustering")
            # msg = Float32MultiArray()
            # msg.data = list(centroid_dict.values())
            # self.publisher_.publish(msg)


            # centroid = {}
            # a= list(valid_clusters.items())
            # b= list(valid_clusters.keys())
            # for i in a:
            #     centroid[b]= sum(i)/len(i)
            # print(centroid)
            # print()

            # mean_values = {}
            # for label, values in valid_clusters.items():
            #     mean_values[label] = np.mean(values)
            # print(mean_values)
            



            # print(self.prev-self.curr)



        # self.previous_ranges = ranges_cart
            












        # for i in range(len(ranges_cart)):
        #     # Calculate the velocity between consecutive points
        #     if i > 0:
        #         distance = abs(ranges_cart[i] - ranges_cart[i-1])
        #         velocity = distance  # Simplified velocity calculation (change in distance)
                
        #         # Check if the point belongs to the current person cluster
        #         if velocity <= max_velocity and distance <= max_distance:
        #             person_cluster.append((ranges_cart[i]))
        #         else:
        #             # Check if the cluster size is sufficient to consider it a person
        #             if len(person_cluster) > min_points:
        #                 detected_people.append(person_cluster)
        #             person_cluster = []  # Reset cluster
        
        # return detected_people
    
    def polar_to_cartesian(self, range_data, angle_min, angle_increment):
        cartesian_points = []
        for i in range(len(range_data)):
            angle = angle_min + i * angle_increment
            x = range_data[i] * math.cos(angle)
            y = range_data[i] * math.sin(angle)
            cartesian_points.append(("{:.3f}".format(x), "{:.3f}".format(y)))
        
        return cartesian_points
        
def main(args=None):
    rclpy.init(args=args)
    person_tracker = PersonTracker()
    rclpy.spin(person_tracker)

    person_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import cv2
import math

from scipy.ndimage import gaussian_filter

class EdgeDetectionNode(Node):
    def __init__(self):
        super().__init__('edge_detection_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize the plot
        self.fig, self.axs = plt.subplots(2)
        self.fig.show()
        self.fig.canvas.draw()
    # Function to calculate the slope between two points
    def calculate_slope(self, p1, p2):
        return np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

    # Function to detect corners based on angle changes
    def detect_corners(self, points, angle_threshold=np.pi/6):
        points_ = points[~np.isnan(points).any(axis=1)]
        corners = []
        start_point = points_[0]
        end_point = points_[-1]
        
        # Compute slopes between consecutive points
        slopes = [self.calculate_slope(points_[i], points_[i+1]) for i in range(len(points_)-1)]
        
        # Detect significant changes in slope
        for i in range(2, len(slopes)):
            angle_change = abs(slopes[i] - slopes[i-1])
            print("Slop: ", angle_change)
            if angle_change > angle_threshold:  # Significant change in angle
                corners.append(points_[i])
        
        return start_point, corners, end_point

    def scan_callback(self, msg):
        # Extract laser scan data
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment
        # Filter out invalid or noisy data
        # print(ranges)
        ranges = self.filter_data(ranges, 3.5)
        ranges = gaussian_filter(ranges, sigma=1)
        # Convert polar coordinates to Cartesian
        points = self.polar_to_cartesian(ranges, angle_min, angle_increment)
        # print(f"Length range and list: {len(ranges), len(points)}")

        # Compute derivative and find peaks
        jumps = self.compute_derivative(ranges, 0.1)
        boxes, cylinder_boundaries, cartesian_boundaries = self.find_cylinders(ranges, jumps, 100,1, angle_min, angle_increment)
        # print("boundry", cylinder_boundaries)
        start = points[cylinder_boundaries[0][0]+1]
        end = points[cylinder_boundaries[0][1]]
        print(f"Length: {self.calculate_edge_length(start, end)}")
        start_end_points = [start, end]
        # Update the plot with the result
        # cylinder_boundaries_cart = self.convert_break_points_to_cartesian(ranges, cylinder_boundaries, angle_min, angle_increment)
        self.plot_edge(points, jumps, ranges, boxes, cylinder_boundaries, start_end_points)


    def plote_points_to_image(self,points):
        image = np.full((500, 500, 3), 255, dtype = np.uint8)   
        points_temp_ = points*100
        for i in range(len(points_temp_)-1):
            point_1 = points_temp_[i]
            point_2 = points_temp_[i+1]
            if ~np.isnan(point_1[0]) and ~np.isnan(point_2[0]):
                # print(point_1[i][0]+250,point_1[i][1])
                cv2.line(image, (int(point_1[0])+250,int(point_1[1])), (int(point_2[0])+250,int(point_2[1])),(0,0,0), 1)
                # cv2.circle(image,(int(point[0])+250,int(point[1])),1,(0,0,0),-1)

        cv2.imshow("output",image)
        cv2.waitKey(5)

    def calculate_edge_length(self, p1, p2):
        """Calculate the total length of the detected edge."""
        distance = np.linalg.norm(p2 - p1)  # Euclidean distance
        return distance

    def polar_to_cartesian_1(self,angle, distance):
        """Convert polar coordinates (angle, distance) to Cartesian (x, y)."""
        x = distance * np.cos(angle)
        y = distance * np.sin(angle)
        return x, y

    def find_cylinders(self, scan, scan_derivative, jump, min_dist, angle_min, angle_increment):
        cylinder_list = []
        cylinder_boundaries = []
        cartesian_boundaries = []
        on_cylinder = False
        sum_ray, sum_depth, rays = 0.0, 0.0, 0
        start_index = None  # To track the start point of a cylinder

        for i in range(len(scan_derivative)):
            # Start of a cylinder (negative jump)
            if scan_derivative[i] < -jump:
                if not on_cylinder:
                    on_cylinder = True
                    start_index = i  # Record start index
                else:
                    sum_ray, sum_depth, rays = 0.0, 0.0, 0

            # End of a cylinder (positive jump)
            elif scan_derivative[i] > jump and rays > 0:
                cylinder_list.append((sum_ray / rays, sum_depth / rays))
                cylinder_boundaries.append((start_index, i))  # Record start and end indices
                
                # Convert boundaries to Cartesian
                start_angle = angle_min + start_index * angle_increment
                end_angle = angle_min + i * angle_increment
                start_cartesian = self.polar_to_cartesian_1(start_angle, scan[start_index])
                end_cartesian = self.polar_to_cartesian_1(end_angle, scan[i])
                cartesian_boundaries.append(start_cartesian)
                cartesian_boundaries.append(end_cartesian)
                
                sum_ray, sum_depth, rays = 0.0, 0.0, 0
                on_cylinder = False

            # Accumulate data if currently on a cylinder
            if on_cylinder and scan[i] > min_dist:
                sum_ray += i
                sum_depth += scan[i]
                rays += 1

        return np.array(cylinder_list), cylinder_boundaries, cartesian_boundaries

    def compute_derivative(self, scan, min_dist):
        jumps = [0]
        for i in range(1, len(scan) - 1):
            l = scan[i-1]
            r = scan[i+1]
            if np.isnan(l):
                l = 6
            if np.isnan(r):
                r = 6
            if (l > min_dist) and (r > min_dist):
                derivative = ((r - l) / 2.0)*100
                jumps.append((derivative))
            else:
                jumps.append(0)
        jumps.append(0)
        return jumps


    def filter_data(self, ranges, range_max):
        """Filter out invalid or noisy data."""
        ranges[ranges > range_max] = np.nan
        return ranges

    def polar_to_cartesian(self, ranges, angle_min, angle_increment):
        """Convert polar coordinates (ranges and angles) to Cartesian coordinates."""
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        return np.vstack((x, y)).T

    def convert_break_points_to_cartesian(self, ranges, break_points, angle_min, angle_increment):
        list_points = []
        for point in break_points:
            for i in range(len(point)):
                index = point[i]
                depth  = ranges[index]
                # print(index, depth)
                x_1,y_1 = self.polar_to_cartesian_from_index(depth, index, angle_min, angle_increment)
                list_points.append((x_1,y_1))

        return list_points
    def polar_to_cartesian_from_index(self, r, index, angle_min, angle_increment):
        # Calculate the angle (theta) based on index, angle_min, and angle_increment
        theta = angle_min + index * angle_increment
        
        # Convert to Cartesian coordinates
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y

    def plot_edge(self, points, jumps, scan, center_points, start_finish_point, start_finish_point_cart):
        """Update the plot with the entire scan data and highlight the detected edge."""
        # print(points)
        self.axs[0].clear()
        self.axs[1].clear()

        x, y = points[:, 0], points[:, 1]
        x = np.append(x,0)
        y = np.append(y,0)
        self.axs[0].plot(x, y, 'b.', label="Scan Points")
        temp_ = np.array(start_finish_point_cart)
        x, y = temp_[:, 0], temp_[:, 1]
        self.axs[0].plot(x,y, 'r-', label="Start Finish Point")

        temp_ = np.array(center_points)
        x, y = temp_[:, 0], temp_[:, 1]
        x = np.append(x,0)
        y = np.append(y,0)
        self.axs[1].plot(x, y, 'r*', label="center object")
        x_sf = []
        y_sf = []
        for point in start_finish_point:
            x_sf.append(point[0])
            x_sf.append(point[1])
            y_sf.append(0)
            y_sf.append(0)
        self.axs[1].plot(x_sf, y_sf, 'y*', label="Start Finish Point")

        self.axs[1].plot(jumps, label="Jumps")
        self.axs[1].plot(scan, 'g-' ,label="ranges")

        self.axs[0].set_xlabel('X (meters)')
        self.axs[0].set_ylabel('Y (meters)')
        self.axs[0].set_title('Edge Detection')
        self.axs[0].legend()

        self.axs[1].set_xlabel('Index')
        self.axs[1].set_ylabel('Jump')
        self.axs[1].set_title('Derivative of Ranges')
        self.axs[1].legend()

        self.fig.canvas.draw()
        plt.pause(0.01)
        # plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = EdgeDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

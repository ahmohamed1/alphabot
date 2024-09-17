#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt

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

    def scan_callback(self, msg):
        # Extract laser scan data
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        # Filter out invalid or noisy data
        ranges = self.filter_data(ranges, 3.5)
        ranges = gaussian_filter(ranges, sigma=1)
        # Convert polar coordinates to Cartesian
        points = self.polar_to_cartesian(ranges, angle_min, angle_increment)

        # Compute derivative and find peaks
        jumps, point_list = self.compute_derivative(ranges, 0.1)
        boxes, cylinder_boundaries = self.find_cylinders(ranges, jumps, 100,1)
        # print(cylinder_boundaries)
        # Update the plot with the result
        self.plot_edge(points, jumps, ranges, boxes, cylinder_boundaries)

    def calculate_edge_length(self, breaking_points):
        """Calculate the total length of the detected edge."""
        total_length = 0.0
        for i in range(1, len(breaking_points)):
            p1 = np.array(breaking_points[i - 1])
            p2 = np.array(breaking_points[i])
            distance = np.linalg.norm(p2 - p1)  # Euclidean distance
            total_length += distance
        return total_length

    def find_cylinders(self, scan, scan_derivative, jump, min_dist):
        cylinder_list = []
        cylinder_boundaries = []
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
                cylinder_boundaries.append((start_index,0))  # Record start and end indices
                cylinder_boundaries.append((i,0))  # Record start and end indices
                sum_ray, sum_depth, rays = 0.0, 0.0, 0
                on_cylinder = False

            # Accumulate data if currently on a cylinder
            if on_cylinder and scan[i] > min_dist:
                sum_ray += i
                sum_depth += scan[i]
                rays += 1

        return np.array(cylinder_list), np.array(cylinder_boundaries)

    def compute_derivative(self, scan, min_dist):
        jumps = [0]
        index_list = []
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
                if abs(derivative) > 100:
                    index_list.append((i,0))
            else:
                jumps.append(0)
        jumps.append(0)
        index_list = index_list[1:-1]
        return jumps, np.array(index_list)


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

    def polar_to_cartesian_from_index(self, r, index, angle_min, angle_increment):
        # Calculate the angle (theta) based on index, angle_min, and angle_increment
        theta = angle_min + index * angle_increment
        
        # Convert to Cartesian coordinates
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        return x, y

    def plot_edge(self, points, jumps, scan, breaking_points, start_finish_point):
        """Update the plot with the entire scan data and highlight the detected edge."""
        # print(points)
        self.axs[0].clear()
        self.axs[1].clear()

        x, y = points[:, 0], points[:, 1]
        x = np.append(x,0)
        y = np.append(y,0)
        self.axs[0].plot(x, y, 'b.', label="Scan Points")

        x, y = breaking_points[:, 0], breaking_points[:, 1]
        x = np.append(x,0)
        y = np.append(y,0)
        self.axs[1].plot(x, y, 'r*', label="center object")

        x, y = start_finish_point[:, 0], start_finish_point[:, 1]
        x = np.append(x,0)
        y = np.append(y,0)
        self.axs[1].plot(x, y, 'y*', label="Start Finish Point")

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
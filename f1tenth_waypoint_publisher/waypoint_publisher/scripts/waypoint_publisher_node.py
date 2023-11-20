#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from waypoint_publisher.msg import WayPoints, WayPoint
import csv


class WaypointPublisherNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('waypoint_publisher_node')
        # Create a publisher to publish control commands to the car.
        self.waypoint_pub = self.create_publisher(WayPoints, "waypoints", 10)
        self.declare_parameter("waypoint_file")
        wp_f = self.get_parameter("waypoint_file").get_parameter_value().string_value
        print(wp_f)
        with open(wp_f, "r") as f:
            csv_reader = csv.reader(f)
            self.waypoints = WayPoints()
            for row in csv_reader:
                waypt = WayPoint()
                waypt.x     = float(row[0])
                waypt.y     = float(row[1])
                waypt.v     = float(row[2])
                waypt.theta = float(row[3])
                self.waypoints.waypoints.append(waypt)
    
    def publish_waypoints(self):
        self.waypoint_pub.publish(self.waypoints)
        self.get_logger().info("Published waypoints")

def main(args=None):
    rclpy.init(args=args)
    waypoint_pub = WaypointPublisherNode()
    waypoint_pub.publish_waypoints()
    rclpy.spin(waypoint_pub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    waypoint_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
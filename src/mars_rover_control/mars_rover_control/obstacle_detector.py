#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose

class ObstacleDetector(Node):
    def __init__(self):
        super().__init__('obstacle_detector')

        self.subscription = self.create_subscription(
            Pose,
            '/model/rover_blue/pose',
            self.pose_callback,
            10)

        self.distance_pub = self.create_publisher(Float32, '/distance_of_closest_obstacle', 10)
        self.direction_pub = self.create_publisher(Pose, '/direction_of_closest_obstacle', 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.rover_x = 5.0
        self.rover_y = 2.5

        # World bounds
        self.map_size = 25.0

        # Define static obstacles [(x, y)]
        self.obstacles = [
            (5, 10),    # boulder_1
            (3, 18),    # boulder_2
            (16, 4),    # boulder_3
            (12, 14),   # boulder_4
            (23, 10),   # boulder_5
            (11, 22),   # boulder_6
        ]

    def pose_callback(self, msg):
        self.rover_x = msg.position.x
        self.rover_y = msg.position.y

    def timer_callback(self):
        # Ignore if rover is far off-grid
        if not (0 <= self.rover_x <= self.map_size and 0 <= self.rover_y <= self.map_size):
            self.get_logger().warn(f"Rover out of bounds: ({self.rover_x:.2f}, {self.rover_y:.2f})")
            return

        min_distance = float('inf')
        nearest_obstacle = (0.0, 0.0)

        for ox, oy in self.obstacles:
            dx = ox - self.rover_x
            dy = oy - self.rover_y
            distance = math.sqrt(dx**2 + dy**2) - 1.0  # obstacle radius = 1.0
            if distance < min_distance:
                min_distance = distance
                nearest_obstacle = (dx, dy)

        # Clamp distance to avoid exploding values
        min_distance = max(0.0, min(min_distance, self.map_size))

        # Normalize direction
        dx, dy = nearest_obstacle
        magnitude = math.hypot(dx, dy)
        if magnitude > 1e-6:
            direction_x = dx / magnitude
            direction_y = dy / magnitude
        else:
            direction_x = 0.0
            direction_y = 0.0

        # Publish
        distance_msg = Float32()
        distance_msg.data = float(min_distance)
        self.distance_pub.publish(distance_msg)

        direction_msg = Pose()
        direction_msg.position.x = float(direction_x)
        direction_msg.position.y = float(direction_y)
        self.direction_pub.publish(direction_msg)

        self.get_logger().info(f"Distance: {min_distance:.2f} m, Direction: ({direction_x:.2f}, {direction_y:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

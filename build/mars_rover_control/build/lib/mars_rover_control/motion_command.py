import sys
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32
from datetime import datetime
import math
import numpy as np
from stable_baselines3 import PPO
from .envs.mars_rover_env import MarsRoverEnv

class MotionCommand(Node):
    def __init__(self):
        super().__init__('motion_command')

        self.cmd_vel_pub = self.create_publisher(Twist, '/rover_blue_cmd_vel', 10)
        self.create_subscription(Pose, '/model/rover_blue/pose', self.pose_callback, 10)
        self.create_subscription(Float32, '/distance_of_closest_obstacle', self.distance_callback, 10)
        self.create_subscription(Pose, '/direction_of_closest_obstacle', self.direction_callback, 10)

        self.timer = self.create_timer(1.0, self.timer_callback)

        model_path = os.path.expanduser("~/rl_mars_ws/trained_models/mars_rover/PPO_50000000/best_model")
        custom_objects = {
            "learning_rate": 1e-4,
            "lr_schedule": lambda _: 1e-4,
            "clip_range": 0.2,
        }
        self.policy = PPO.load(model_path, env=MarsRoverEnv(), custom_objects=custom_objects)

        self.rover_pose = None
        self.distance_to_obstacle = None
        self.obstacle_direction = None
        self.target_position = (19.0, 21.0)

        self.file = open('rover_log.txt', 'w')
        self.file.write("timestamp, rover_x, rover_y, heading, desired_heading, linear, angular, distance_to_target, distance_to_obstacle, obstacle_dx, obstacle_dy\n")

        self.start_time = self.get_clock().now()
        self.step_count = 0

    def pose_callback(self, msg):
        self.rover_pose = msg

    def distance_callback(self, msg):
        self.distance_to_obstacle = msg.data

    def direction_callback(self, msg):
        self.obstacle_direction = (msg.position.x, msg.position.y)

    def timer_callback(self):
        if self.rover_pose is None or self.distance_to_obstacle is None or self.obstacle_direction is None:
            self.get_logger().info('Waiting for all data...')
            return

        x = self.rover_pose.position.x
        y = self.rover_pose.position.y
        tx, ty = self.target_position
        dx, dy = self.obstacle_direction
        d_target = math.sqrt((tx - x)**2 + (ty - y)**2)

        obs = np.array([x, y, tx, ty, self.distance_to_obstacle, dx, dy], dtype=np.float32)
        action, _ = self.policy.predict(obs, deterministic=True)
        ux, uy = action

        q = self.rover_pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        desired_heading = math.atan2(uy, ux)
        angle_error = math.atan2(math.sin(desired_heading - yaw), math.cos(desired_heading - yaw))

        cmd = Twist()
        if abs(angle_error) > math.radians(1):
            cmd.linear.x = 0.0
            cmd.angular.z = max(min(angle_error, math.radians(10)), -math.radians(10))
        else:
            cmd.linear.x = 0.5
            cmd.angular.z = 0.0

        self.cmd_vel_pub.publish(cmd)

        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        self.file.write(f"{timestamp}, {x:.2f}, {y:.2f}, {yaw:.2f}, {desired_heading:.2f}, {cmd.linear.x:.2f}, {cmd.angular.z:.2f}, {d_target:.2f}, {self.distance_to_obstacle:.2f}, {dx:.2f}, {dy:.2f}\n")
        self.file.flush()

        self.step_count += 1
        self.get_logger().info(f"[Step {self.step_count}] Action: [{ux:.2f}, {uy:.2f}], Linear: {cmd.linear.x:.2f}, Angular: {cmd.angular.z:.2f}")

        ROVER_RADIUS = 1.12
        BOULDER_RADIUS = 1.0
        if self.distance_to_obstacle < (ROVER_RADIUS + BOULDER_RADIUS):
            self.get_logger().info("Rover hit a boulder.")
            self.destroy_node()
            return

        if (tx - 1 <= x <= tx + 1) and (ty - 1 <= y <= ty + 1):
            self.get_logger().info("Rover reached the target.")
            self.destroy_node()
            return

        if not (0 <= x <= 25) or not (0 <= y <= 25):
            self.get_logger().warn(f"Rover out of bounds: ({x:.2f}, {y:.2f})")
            self.destroy_node()
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > 200:
            self.get_logger().warn("Episode timed out (200 seconds).")
            self.destroy_node()
            return

    def destroy_node(self):
        if hasattr(self, 'file') and self.file:
            self.file.close()
            self.get_logger().info('Motion Command Node shutting down and file saved.')

            try:
                import pandas as pd
                import matplotlib.pyplot as plt

                log_path = "rover_log.txt"
                if not os.path.exists(log_path):
                    self.get_logger().error(f"Log file {log_path} not found.")
                    return

                log_df = pd.read_csv(log_path)
                log_df.columns = [col.strip() for col in log_df.columns]

                plots_dir = "plots"
                os.makedirs(plots_dir, exist_ok=True)

                timestamps = pd.to_datetime(log_df["timestamp"]).to_numpy()
                x_vals = log_df["rover_x"].to_numpy()
                y_vals = log_df["rover_y"].to_numpy()
                heading = log_df["heading"].to_numpy()
                desired_heading = log_df["desired_heading"].to_numpy()
                obstacle_dist = log_df["distance_to_obstacle"].to_numpy()

                # Plot 1: Rover trajectory
                plt.figure()
                plt.plot(x_vals, y_vals, marker='o')
                plt.title("Rover Trajectory")
                plt.xlabel("X position (m)")
                plt.ylabel("Y position (m)")
                plt.axis("equal")
                plt.grid(True)
                plt.savefig(os.path.join(plots_dir, "rover_trajectory.png"))
                plt.close()

                # Plot 2: Heading comparison
                plt.figure()
                plt.plot(timestamps, heading, label="Rover Heading")
                plt.plot(timestamps, desired_heading, label="NN Desired Heading")
                plt.title("Orientation vs NN Direction")
                plt.xlabel("Time")
                plt.ylabel("Radians")
                plt.legend()
                plt.grid(True)
                plt.xticks(rotation=45)
                plt.tight_layout()
                plt.savefig(os.path.join(plots_dir, "rover_heading_comparison.png"))
                plt.close()

                # Plot 3: Distance to obstacle
                plt.figure()
                plt.plot(timestamps, obstacle_dist)
                plt.title("Distance to Closest Obstacle Over Time")
                plt.xlabel("Time")
                plt.ylabel("Distance (m)")
                plt.grid(True)
                plt.xticks(rotation=45)
                plt.tight_layout()
                plt.savefig(os.path.join(plots_dir, "obstacle_distance_over_time.png"))
                plt.close()

                self.get_logger().info("Plots saved in 'plots/' directory.")

            except Exception as e:
                self.get_logger().error(f"Failed to generate plots: {e}")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotionCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Pose
# from ros_ign_interfaces.srv import SpawnEntity
# import random
# import math
# import time

# MAP_SIZE = 25
# ROVER_SIZE = 2.0
# TARGET_SIZE = 2.0
# BOULDER_RADIUS = 1.0
# NUM_BOULDERS = 6

# def euclidean(p1, p2):
#     return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

# class EntitySpawner(Node):
#     def __init__(self):
#         super().__init__('spawn_entities')
#         self.cli = self.create_client(SpawnEntity, '/spawn_entity')
#         while not self.cli.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Waiting for /spawn_entity service...')
#         time.sleep(2.0)  # Extra wait for Gazebo to fully initialize
#         self.spawn_all()

#     def spawn_all(self):
#         positions = {}

#         # Sample rover
#         while True:
#             rover = (random.uniform(ROVER_SIZE, MAP_SIZE - ROVER_SIZE),
#                      random.uniform(ROVER_SIZE, MAP_SIZE - ROVER_SIZE))
#             if self.valid(rover, positions.values(), min_dist=6.8):
#                 positions['rover'] = rover
#                 self.get_logger().info(f"Rover position: {rover}")
#                 break

#         # Sample target
#         while True:
#             target = (random.uniform(TARGET_SIZE, MAP_SIZE - TARGET_SIZE),
#                       random.uniform(TARGET_SIZE, MAP_SIZE - TARGET_SIZE))
#             if euclidean(target, rover) >= 12.5:
#                 if self.valid(target, positions.values(), min_dist=6.8):
#                     positions['target'] = target
#                     self.get_logger().info(f"Target position: {target}")
#                     break

#         # Sample boulders
#         boulders = []
#         while len(boulders) < NUM_BOULDERS:
#             candidate = (random.uniform(BOULDER_RADIUS, MAP_SIZE - BOULDER_RADIUS),
#                          random.uniform(BOULDER_RADIUS, MAP_SIZE - BOULDER_RADIUS))
#             if self.valid(candidate, positions.values(), min_dist=6.8) and \
#                self.valid(candidate, boulders, min_dist=6.8):
#                 boulders.append(candidate)
#                 self.get_logger().info(f"Boulder {len(boulders)} position: {candidate}")
#         positions['boulders'] = boulders

#         # Spawn all models
#         self.spawn_model('rover_blue', positions['rover'], 'rover')
#         self.spawn_model('target_plane', positions['target'], 'target')
#         for i, pos in enumerate(boulders):
#             self.spawn_model(f'boulder_{i+1}', pos, 'boulder')

#         self.get_logger().info('All entities spawned successfully.')

#     def valid(self, candidate, existing, min_dist):
#         return all(euclidean(candidate, p) >= min_dist for p in existing)

#     def spawn_model(self, name, pos, type_):
#         req = SpawnEntity.Request()
#         req.name = name
#         req.xml = self.get_sdf_string(type_)
#         req.robot_namespace = name
#         req.initial_pose.position.x = float(pos[0])
#         req.initial_pose.position.y = float(pos[1])
#         req.initial_pose.position.z = 0.05  # Raised height to avoid flickering
#         req.initial_pose.orientation.w = 1.0
#         future = self.cli.call_async(req)
#         rclpy.spin_until_future_complete(self, future)
#         if future.result() is not None:
#             self.get_logger().info(f"Spawned {name} at {pos}")
#         else:
#             self.get_logger().error(f"Failed to spawn {name}")

#     def get_sdf_string(self, type_):
#         if type_ == 'rover':
#             path = '/home/david/ros2_mars_ws/src/mars_rover_control/models/rover_blue.sdf'
#         elif type_ == 'target':
#             path = '/home/david/ros2_mars_ws/src/mars_rover_control/models/target_plane.sdf'
#         elif type_ == 'boulder':
#             path = '/home/david/ros2_mars_ws/src/mars_rover_control/models/boulder.sdf'
#         else:
#             raise ValueError("Unknown type")
#         with open(path, 'r') as f:
#             return f.read()

# def main(args=None):
#     rclpy.init(args=args)
#     node = EntitySpawner()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

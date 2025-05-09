# mars_rover_env.py

import gymnasium as gym
import numpy as np
from gymnasium import spaces
import matplotlib.pyplot as plt
import pygame



class MarsRoverEnv(gym.Env):
    """
    Gymnasium-compatible Mars Rover environment for RL training.
    """
    metadata = {"render_modes": ["human"], "render_fps": 1}

    def __init__(self, render_mode=None):
        super(MarsRoverEnv, self).__init__()

        self.render_mode = render_mode
        self._figure = None  # placeholder for matplotlib fig

        self.map_size = 25.0
        self.rover_radius = 1.0
        self.target_radius = 1.0
        self.boulder_radius = 1.0
        self.num_boulders = 6
        self.max_steps = 200
        self.velocity = 0.5
        self.dt = 1.0

        # Action: unit vector [ux, uy]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)

        # Observation: [x, y, xT, yT, d, dx, dy]
        self.observation_space = spaces.Box(
            low=np.array([0, 0, 0, 0, 0, -1, -1], dtype=np.float32),
            high=np.array([25, 25, 25, 25, 25, 1, 1], dtype=np.float32),
        )

        # Tracking
        self.termination_stats = {
            "reached_target": 0,
            "collision": 0,
            "out_of_bounds": 0,
            "timeout": 0
        }

        self.render_mode = False  # Toggle to enable matplotlib rendering
        self.reset()

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0

        while True:
            self.rover_pos = self._random_position()
            self.target_pos = self._random_position()
            if np.linalg.norm(self.rover_pos - self.target_pos) >= 12.5:
                break

        self.boulders = []
        while len(self.boulders) < self.num_boulders:
            pos = self._random_position()
            valid = (
                np.linalg.norm(pos - self.rover_pos) > 6.8 and
                np.linalg.norm(pos - self.target_pos) > 6.8 and
                all(np.linalg.norm(pos - b) > 6.8 for b in self.boulders)
            )
            if valid:
                self.boulders.append(pos)
        self.boulders = np.array(self.boulders)

        return self._get_obs(), {}

    def step(self, action):
        action = np.array(action)
        if np.linalg.norm(action) != 0:
            action = action / np.linalg.norm(action)

        self.rover_pos += self.velocity * action * self.dt
        self.steps += 1

        done, reason = self._check_done()
        reward = self._compute_reward(done, reason)

        terminated = reason in ["reached_target", "collision", "out_of_bounds"]
        truncated = reason == "timeout"
        info = {"reason": reason}

        if done:
            self.termination_stats[reason] += 1
            print(f"[Episode Ended] Reason: {reason:<15} | Steps: {self.steps:<3} | Reward: {reward:+.2f}")
            if self.render_mode:
                self.render()

        return self._get_obs(), reward, terminated, truncated, info

    def _get_obs(self):
        min_dist = float('inf')
        dir_vec = np.array([0.0, 0.0])
        for b in self.boulders:
            vec = b - self.rover_pos
            dist = np.linalg.norm(vec) - self.boulder_radius
            if dist < min_dist:
                min_dist = dist
                dir_vec = vec / (np.linalg.norm(vec) + 1e-8)

        return np.array([
            *self.rover_pos,
            *self.target_pos,
            min_dist,
            *dir_vec
        ], dtype=np.float32)

    def _check_done(self):
        if not (0 <= self.rover_pos[0] <= self.map_size and 0 <= self.rover_pos[1] <= self.map_size):
            return True, "out_of_bounds"
        if np.linalg.norm(self.rover_pos - self.target_pos) <= self.target_radius:
            return True, "reached_target"
        for b in self.boulders:
            if np.linalg.norm(self.rover_pos - b) <= self.boulder_radius:
                return True, "collision"
        if self.steps >= self.max_steps:
            return True, "timeout"
        return False, None

    def _compute_reward(self, done, reason):
        if done:
            if reason == "reached_target":
                return 100.0
            elif reason == "collision":
                return -100.0
            elif reason == "out_of_bounds":
                return -50.0
            elif reason == "timeout":
                return -10.0
        return -np.linalg.norm(self.rover_pos - self.target_pos) * 0.1

    def _random_position(self):
        return np.random.uniform(low=0.0, high=self.map_size, size=(2,))

    def render(self):
        if not hasattr(self, "window"):
            pygame.init()
            self.window_size = 600
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
            self.clock = pygame.time.Clock()
            self.trail = []

        self.window.fill((255, 255, 255))
        scale = self.window_size / self.map_size

        # Draw boulders
        for b in self.boulders:
            pygame.draw.circle(
                self.window, (100, 100, 100),
                (int(b[0] * scale), self.window_size - int(b[1] * scale)),
                int(self.boulder_radius * scale))

        # Draw target as square
        rect_size = int(2 * self.target_radius * scale)
        rect_origin = (int((self.target_pos[0] - self.target_radius) * scale),
                    self.window_size - int((self.target_pos[1] + self.target_radius) * scale))
        pygame.draw.rect(self.window, (0, 255, 0), (*rect_origin, rect_size, rect_size))

        # Draw rover
        rover_pos_px = (int(self.rover_pos[0] * scale), self.window_size - int(self.rover_pos[1] * scale))
        pygame.draw.circle(self.window, (0, 0, 255), rover_pos_px, int(self.rover_radius * scale))

        # Draw trail
        self.trail.append(rover_pos_px)
        if len(self.trail) > 1:
            pygame.draw.lines(self.window, (0, 0, 255), False, self.trail, 2)

        pygame.display.flip()
        pygame.event.pump()
        self.clock.tick(1)  # 1 Hz to match simulation time step



    def close(self):
        pass

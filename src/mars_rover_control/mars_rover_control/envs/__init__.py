from gymnasium.envs.registration import register

register(
    id='MarsRover-v0',
    entry_point='envs.mars_rover_env:MarsRoverEnv',
    max_episode_steps=200,
)

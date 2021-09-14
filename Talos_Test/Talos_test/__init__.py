from gym.envs.registration import register

register(
     id='Talos_Test-v0',
     entry_point='Talos_test.envs:Simple_Env',
     max_episode_steps=3000,
)
import time
import torch
import gym
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2
import sys
sys.path.append('./')
import Talos_test.envs

def main():
    env = gym.make('Talos_Test-v0')
    model = PPO2(MlpPolicy, env, verbose=1, tensorboard_log="./tensorboard/").learn(total_timesteps=5000000)
    model.save('PPO_Talos_Test-vo')
    del model
    env.close()


if __name__ =='__main__':
    main()

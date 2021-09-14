
import time
import torch
import gym
import pybullet_envs
from stable_baselines.common.policies import MlpPolicy
from stable_baselines import PPO2
from stable_baselines.common.callbacks import CheckpointCallback
from stable_baselines.common.vec_env import DummyVecEnv, SubprocVecEnv

import sys
sys.path.append('./')

import Talos_test.envs


from stable_baselines.common import set_global_seeds
import gym
def make_env(env_id, rank, seed=0):
    #Utility function for multiprocessed env.
    #:param env_id: (str) the environment ID
    #:param seed: (int) the inital seed for RNG
    #:param rank: (int) index of the subprocess
    def _init():
        env = gym.make(env_id)
        # Important: use a different seed for each environment
        print("===========================================> SETTING SEED : ",seed+rank)
        env.seed(seed + rank)
        return env
    set_global_seeds(seed)
    return _init


def main():
    """
    from Talos_test.resources.Talos import Talos
    import pybullet as p
    import pybullet_data
    client = p.connect(p.GUI)
    #basic setup
    robotStartPosition = [0.0, 0.0, 3]  # 1.08]
    robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    robotId = p.loadURDF("talos_reduced.urdf",robotStartPosition, robotStartOrientation, useFixedBase = True)
    #talos = Talos(client)
    input("...")
    """
    print("=========== Create process")
    n_procs = 3
    env_id = 'Talos_Test-v0'
    if n_procs == 1:
        # if there is only one process, there is no need to use multiprocessing
        env = [lambda: gym.make(env_id)]
        envVec = DummyVecEnv(env)
    else:
        env = [make_env(env_id, i) for i in range(n_procs)]
        envVec = SubprocVecEnv(env, start_method='spawn')
    #input("process created ...")
    print("=========== Create model")
    model = PPO2(MlpPolicy, envVec, verbose=1, tensorboard_log="./tensorboard/",
                 n_steps=2048, nminibatches=64, noptepochs=10, cliprange=0.2, learning_rate=1.0e-4,
                 gamma=0.95)
    print("=========== Create callback")
    checkpoint_callback = CheckpointCallback(save_freq=20000, save_path='./logs/',
                                             name_prefix="model_")
    print("=========== Learn")
    model.learn(total_timesteps=1000000, callback=checkpoint_callback)
    input("Training over...")
    del model
    env.close()


if __name__ =='__main__':
    main()

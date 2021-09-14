import gym
import numpy as np
import math
import pybullet as p
from Talos_test.resources.Talos_Left_Arm import Talos
from Talos_test.resources.Plane import Plane


class Simple_Env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=1,
            high=1,
            shape=(30),
            dtype=np.float32  # normalize the action and obs space.
        )

        self.observation_space = gym.spaces.box.Box(
            low=1,
            high=1,
            shape=(30, 2),
            dtype=np.float32
        )


        self.client = p.connect(p.GUI)
        dt = 1e-3
        p.setTimeStep(dt, self.client)
        self.talos = None
        self.done = False
        self.reset()

    def step(self, action):
        reward = 0
        self.talos.apply_action(action)
        p.stepSimulation()
        talos_ob = self.talos.get_observation()
        left_Arm= talos_ob[5]
        Pos = left_Arm[0]
        Vel = left_Arm[1]
        if (Pos>=0, Pos<=0.01):
            self.done = False
            reward +=1
        if (Vel == 0):
            self.done = False
            reward +=0.5
        ob = np.array(talos_ob)
        info = {}
        return ob, reward, self.done, info

    def reset(self):
        p.resetSimulation(self.client)
        p.setGravity(0, 0, -9.81)
        Plane(self.client)
        self.talos = Talos(self.client)
        self.done = False
        talos_ob = self.talos.get_observation()
        return np.array(talos_ob, dtype=np.float32)

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)

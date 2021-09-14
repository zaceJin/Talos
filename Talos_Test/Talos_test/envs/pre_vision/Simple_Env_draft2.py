import gym
import numpy as np
import math
import pybullet as p
from Talos_test.resources.Talos_Left_Arm import Talos
from Talos_test.resources.Plane import Plane


class Simple_Env(gym.Env):
    metadata = {'render.modes': ['human']}

    init_Pos = []
    #step = 0
    def __init__(self):
        self.action_space = gym.spaces.box.Box(
            low=-1,
            high=1,
            shape=(30,),
            dtype=np.float32  # normalize the action and obs space.
        )

        self.observation_space = gym.spaces.box.Box(
            low=-1,
            high=1,
            shape=(60,),
            dtype=np.float32
        )

        self.Count = 0
        self.client = p.connect(p.GUI)
        dt = 1e-3
        p.setTimeStep(dt, self.client)
        self.plane = Plane(self.client)
        self.talos = Talos(self.client)
        self.done = False
        #print("** 1 **")
        #print("Test: ",self.talos.get_observation())
        #print("** 2 **")
        self.reset()
        pass

    def step(self, action):
        rewards = 0
        self.talos.apply_action(action)
        p.stepSimulation()
        talos_ob = self.talos.get_observation()
        #left_Arm = talos_ob[5]
        #Pos4 = talos_ob[4]
        #Pos5 = talos_ob[5]

        #Vel4 = talos_ob[35]
        if (talos_ob[4]>=-0.01 , talos_ob[4]<=0.01):
            print(talos_ob[4])
            self.done = False
            rewards +=1
        if (talos_ob[5]>=1.35 and talos_ob[5] <=1.37):
            self.done = False
            rewards +=1
        if (talos_ob[6] >= -0.01 and talos_ob[6] <= 0):
            self.done = False
            rewards += 1
        if (talos_ob[7] >= -0.01 and talos_ob[5] <= 0):
            self.done = False
            rewards += 1
        if (talos_ob[8] >= -0.01 and talos_ob[8] <= 0):
            self.done = False
            rewards += 1
        if (talos_ob[9] >= -0.17 and talos_ob[9] <= -0.16):
            self.done = False
            rewards += 1
        if (talos_ob[10] >= -0.005 and talos_ob[10] <= 0):
            self.done = False
            rewards += 1
        '''if (Vel == 0):
            self.done = False
            reward +=0.5'''
        obs = np.array(talos_ob)
        info = {}
        self.Count +=1
        print("Step: ", self.Count, "rewards:" , rewards)
        return obs, rewards, self.done, info

    def reset(self):
        # TO DO : Reset joints etc, https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3
        #         resetJointState and resetBasePositionAndOrientation, resetBaseVelocity
        p.resetBasePositionAndOrientation(self.talos.robotId, self.talos.robotStartPosition,self.talos.robotStartOrientation, self.client)
        p.resetBaseVelocity(self.talos.robotId, [0.0,0.0,0.0],[0.0,0.0,0.0], self.client)
        for i in self.talos.controlledJoints:
            p.resetJointState(self.talos.robotId,i,  0.0, 0.0, self.client)
        self.done = False
        talos_ob = self.talos.get_observation()
        #self.step = 0
        #print("Test: ",self.talos.get_observation())
        #print("talos obs : ",talos_ob)
        return np.array(talos_ob, dtype=np.float32)

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)

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
            low=-1,
            high=1,
            shape=(7,),
            dtype=np.float32  # normalize the action and obs space.
        )

        self.observation_space = gym.spaces.box.Box(
            low=-1,
            high=1,
            shape=(14,),
            dtype=np.float32
        )


        self.client = p.connect(p.GUI)
        dt = 1e-3#1e-3
        p.setTimeStep(dt, self.client)
        self.plane = Plane(self.client)
        self.talos = Talos(self.client)
        #print("** 1 **")
        #print("Test: ",self.talos.get_observation())
        #print("** 2 **")
        # VARIABLES
        self.index_step = 0
        self.HORIZON = 1000
        # RESET
        self.reset()
        pass

    def step(self, action):
        self.index_step +=1
        # Take actions
        # TO DO
        #actionScaled =
        # Get previous joint pos and vel
        jointPositionsPrevious, jointVelocitiesPrevious = self.talos.getJointsValues()
        # Unnormalized action
        joints_Torques = self.calculate_Torques(action)
        self.talos.applyAction(joints_Torques)
        self.done = False

        p.stepSimulation()
        p.stepSimulation()
        p.stepSimulation()
        p.stepSimulation()
        p.stepSimulation()
        '''p.stepSimulation()
        p.stepSimulation()
        p.stepSimulation()
        p.stepSimulation()
        p.stepSimulation()'''

        # Check if done
        jointPositions, jointVelocities = self.talos.getJointsValues()
        indexJoints = [ 4,5,6,7,8,9,10 ]
        boundsJoints = [ [-0.01,0.01], [1.35,1.37], [-0.01,0], [-0.01,0], [-0.01,0], [-0.17,-0.16], [-0.01,0] ] # Joints : 4,5,6,7,8,9,10
        jointsTarget = np.array([ value[0] for value in boundsJoints ])

        #self.done = self.isJointsInBounds(jointPositions, indexJoints, boundsJoints) # Done if all joints are in bounds
        if self.index_step >= self.HORIZON:
            self. done = True
        # Get reward
        reward = self.getReward(jointPositionsPrevious, jointVelocitiesPrevious, jointPositions, jointVelocities, indexJoints, jointsTarget, action)
        # Return result
        obs = self.talos.get_observation_normalized()
        info = {}
        #print("Obs : ",obs)
        done = self.done
        #print("Rewards: ", reward )
        return obs, reward, done, info

    #def checkDone(self, index_step, ):
    def reset(self):
        # TO DO : Reset joints etc, https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#heading=h.2ye70wns7io3
        #         resetJointState and resetBasePositionAndOrientation, resetBaseVelocity
        p.resetBasePositionAndOrientation(self.talos.robotId, self.talos.robotStartPosition,self.talos.robotStartOrientation, self.client)
        p.resetBaseVelocity(self.talos.robotId, [0.0,0.0,0.0],[0.0,0.0,0.0], self.client)
        for i in self.talos.controlledJoints:
            p.resetJointState(self.talos.robotId,i,  0.0, 0.0, self.client)
        talos_ob = self.talos.get_observation()
        self.index_step = 0
        #print("Test: ",self.talos.get_observation())
        #print("Obs : ",talos_ob)
        return np.array(talos_ob, dtype=np.float32)

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)



    #===========================================================================

    def getReward(self, jointPositionsPrevious, jointVelocitiesPrevious, jointPositions, jointVelocities, indexJoints,  jointsTarget, action):
        reward = 0
        # (1) Punish for being far from the target
        leftArm_Pos = np.array([])
        for i in range(len(indexJoints)):
            leftArm_Pos = np.append(leftArm_Pos,jointPositions[i])
        #print(leftArm_Pos)
        obs_left, _ = self.talos.normalizeJoints(leftArm_Pos, jointVelocities)
        obs_target, _ = self.talos.normalizeJoints(jointsTarget, jointVelocities)
        distance1 = np.absolute(obs_left-obs_target)
        rPos = - np.linalg.norm(distance1) / (2*np.linalg.norm(len(obs_left)))  # [-1,0]
        # (2) Punish for taking large action
        obs_Pos, obs_Vel = self.talos.normalizeJoints(jointPositions, jointVelocities)
        obs_Pos_previous, obs_Vel_previous = self.talos.normalizeJoints(jointPositionsPrevious, jointVelocitiesPrevious)
        distance = np.absolute(np.array(obs_Pos)-np.array(obs_Pos_previous))
        rEffort = -np.linalg.norm(distance)/(2*np.linalg.norm(len(obs_Pos))) # [-1,0]
        # Sum of rewards
        reward = 0.9*rPos + 0.1*rEffort # [-1, 0]
        reward = reward / self.HORIZON
        print("Rpos = ", rPos, "rEffort :", rEffort, "Rewards: ", reward)
        return reward

    def isJointsInBounds(self, joint_positions, indexJoints, bound_joints):
        counter_joints = 0
        counter_joints_max = 7
        for i in range(len(indexJoints)):
            if (joint_positions[indexJoints[i]]>=bound_joints[i][0] and joint_positions[indexJoints[i]]<=bound_joints[i][1]):
                counter_joints += 1
        return counter_joints==counter_joints_max


    def calculate_Torques(self, action):
        #Calculate Action state from normalization
        qdes = []
        for i in range (len(action)):
            qdes_i = action[i] * (self.talos.mapJointLimit[i][1]-self.talos.mapJointLimit[i][0]) + self.talos.mapJointLimit[i][0]
            qdes = np.append(qdes,qdes_i)
        vdes = [0.0 for _ in self.talos.controlledJoints]
        #jointPosition, jointVelocities = self.getCurrentState()
        joints_Torques  = self.talos.pdController(qdes, vdes)
        return joints_Torques
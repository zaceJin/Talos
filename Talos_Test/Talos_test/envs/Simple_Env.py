import gym
import numpy as np
import math
import pybullet as p
from Talos_test.resources.Talos import Talos
from Talos_test.resources.Plane import Plane


class Simple_Env(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self):

        self.client = p.connect(p.SHARED_MEMORY)
        if self.client < 0:
            self.client = p.connect(p.GUI)
        #self.client = p.connect(None)#p.DIRECT)
        #self.client = p.connect(p.GUI) # WITH GUI
        dt = 1e-3#1e-3
        p.setTimeStep(dt, self.client)
        self.plane = Plane(self.client)
        self.talos = Talos(self.client)

        # Joints controlled
        self.controlledJoints = self.talos.controlledJoints
        self.mapJointLimit = self.talos.mapJointLimit
        print("Env, number of joints observable : ",len(self.controlledJoints))
        #print("Bounds : ",self.mapJointLimit)
        #input("...")

        # Action and Observation spaces
        self.action_space = gym.spaces.box.Box(
            low=-1,
            high=1,
            shape=(len(self.controlledJoints),),
            dtype=np.float32
        )

        self.observation_space = gym.spaces.box.Box(
            low=-1,
            high=1,
            shape=(len(self.controlledJoints)*2,),
            dtype=np.float32
        )
        
        #print("** 1 **")
        #print("Test: ",self.talos.get_observation())
        #print("** 2 **")
        # VARIABLES
        self.index_step = 0
        self.total_reward = 0.0
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
        for i in range(3):
            p.stepSimulation()
        jointPositions, jointVelocities = self.talos.getJointsValues()
        #self.done = self.isJointsInBounds(jointPositions, controlledJoints, mapJointLimit) # Done if all joints are in bounds
        if self.index_step >= self.HORIZON:
            self. done = True
        # Get reward
        reward = self.getReward(jointPositionsPrevious, jointVelocitiesPrevious, jointPositions, jointVelocities, action)
        self.total_reward += reward
        # Return result
        obs = self.talos.get_observation_normalized()
        info = {}
        #print("Obs : ",obs)
        done = self.done
        #print("Rewards: ", reward )
        if done:
            print("Done, number of steps : ",self.index_step," and reward episode : ",self.total_reward)
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
        self.total_reward = 0.0
        #print("Test: ",self.talos.get_observation())
        #print("Obs : ",talos_ob)
        return np.array(talos_ob, dtype=np.float32)

    def render(self):
        pass

    def close(self):
        p.disconnect(self.client)



    #===========================================================================

    def getReward(self, jointPositionsPrevious, jointVelocitiesPrevious, jointPositions, jointVelocities, action):
        reward = 0
        # Set a target for left arm
        jointsIndices = [11,12,13,14,15,16,17] # This is a target for the left arm
        jointsTarget = []
        for i in jointsIndices:
            jointsTarget.append(self.talos.mapJointLimitAll[i][0]) # Target is the min value of this joint for this exercise
        # Get pos of left arm only
        leftArmPos,leftArmVel = [], []
        leftArmPosPrevious, leftArmVelPrevious = [], []
        for i in jointsIndices:
            index = self.talos.mapJointsToControlled[i] # Index of the joints in the list of controlled joints
            # pos
            leftArmPos.append( jointPositions[index] )
            leftArmPosPrevious.append( jointPositionsPrevious[index] )
            # vel
            leftArmVel.append( jointVelocities[0] )
            leftArmVelPrevious.append( jointVelocitiesPrevious[index] )
        # (1) Punish for being far from the target
        distance = np.linalg.norm( np.array( jointsTarget ) - np.array( leftArmPos ) ) / math.sqrt( len(jointsIndices) )
        rPos = -distance
        #print("Distance to target : ",distance)
        #print("rPos = ",rPos)
        #rPos = - np.linalg.norm(distance1) / (2*np.linalg.norm(len(obs_left)))  # [-1,0]
        # (2) Punish for taking large action
        distance = np.linalg.norm( np.array( leftArmPosPrevious ) - np.array( leftArmPos ) ) / math.sqrt( len(jointsIndices) ) # Value between [0,1]
        rEffort = -distance
        # Sum of rewards
        reward = 0.9*rPos + 0.1*rEffort # [-1, 0]
        reward = reward / self.HORIZON
        #print("Rpos = ", rPos, "rEffort :", rEffort, "Rewards: ", reward)
        return reward

    def isJointsInBounds(self, joint_positions, controlledJoints, bound_joints):
        counter_joints = 0
        counter_joints_max = len(controlledJoints)
        for i in range(len(controlledJoints)):
            if (joint_positions[controlledJoints[i]]>=bound_joints[i][0] and joint_positions[controlledJoints[i]]<=bound_joints[i][1]):
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
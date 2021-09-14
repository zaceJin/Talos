import pybullet as p
import numpy as np
import os
import pybullet_data
import example_robot_data

class Talos:
    def __init__(self,client):
        self.client = client

        #basic setup
        self.robotStartPosition = [0.0, 0.0, 3]  # 1.08]
        self.robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        #load Talos
        #p.setTimeStep(1e-3)
        self.robotId = p.loadURDF("talos_reduced.urdf",self.robotStartPosition, self.robotStartOrientation, useFixedBase = True)

        #Joint Limit
        self.mapJointLimit = [
            [-1.308996939, 1.308996939],        # TORSO 0-1
            [-0.261799387799, 0.785398163397],
            [-0.261799387799, 0.785398163397],  # HEAD 2-3
            [-1.308996939, 1.308996939],
            [-1.57079632679, 0.523598775598],   # LEFT ARM 4-10
            [0.0, 2.87979326579],
            [-2.44346095279, 2.44346095279],
            [-2.35619449019, 0.0],
            [-2.53072741539, 2.53072741539],
            [-1.3962634016, 1.3962634016],
            [-0.698131700798, 0.698131700798],
            [-0.523598775598, 1.57079632679], # RIGHT ARM 11-17
            [-2.87979326579, 0.0],
            [-2.44346095279, 2.44346095279],
            [-2.35619449019, 0.0],
            [-2.53072741539, 2.53072741539],
            [-1.3962634016, 1.3962634016],
            [-0.698131700798, 0.698131700798],
            [-0.349065850399, 1.57079632679],   # LEFT LEG 18-23
            [-0.5236, 0.5236],
            [-2.095, 0.7],
            [0.0, 2.618],
            [-1.309, 0.768],
            [-0.5236, 0.5236],
            [-1.57079632679, 0.349065850399],   # RIGHT LEG 24-29
            [-0.5236, 0.5236],
            [-2.095, 0.7],
            [0.0, 2.618],
            [-1.309, 0.768],
            [-0.5236, 0.5236]
        ]

        self.mapJointVel =  [5.4, 5.4, 1.0, 1.0, 2.7, 3.66, 4.58, 4.58, 1.95, 1.76, 1.76, 2.7, 3.66, 4.58, 4.58, 
                             1.95, 1.76, 1.76, 3.87, 5.8, 5.8, 7.0, 5.8, 4.8, 3.87, 5.8, 5.8, 7.0, 5.8, 4.8]

        nonControlledJoints = [
            "imu_joint",  # Other joints not used
            "rgbd_joint",
            "rgbd_optical_joint",
            "rgbd_depth_joint",
            "rgbd_depth_optical_joint",
            "rgbd_rgb_joint",
            "rgbd_rgb_optical_joint",
            "wrist_left_ft_joint",  # RIGHT GRIPPER
            "wrist_left_tool_joint",
            "gripper_left_base_link_joint",
            "gripper_left_joint",
            "gripper_left_inner_double_joint",
            "gripper_left_fingertip_1_joint",
            "gripper_left_fingertip_2_joint",
            "gripper_left_motor_single_joint",
            "gripper_left_inner_single_joint",
            "gripper_left_fingertip_3_joint",
            "wrist_right_ft_joint",  # LEFT GRIPPER
            "wrist_right_tool_joint",
            "gripper_right_base_link_joint",
            "gripper_right_joint",
            "gripper_right_inner_double_joint",
            "gripper_right_fingertip_1_joint",
            "gripper_right_fingertip_2_joint",
            "gripper_right_motor_single_joint",
            "gripper_right_inner_single_joint",
            "gripper_right_fingertip_3_joint",
            "leg_left_sole_fix_joint",  # LEFT LEG (blocked)
            "leg_right_sole_fix_joint"  # RIGHT LEG (blocked)
        ]

        bulletJointNames = [p.getJointInfo(self.robotId, i)[1].decode() for i in range(p.getNumJoints(self.robotId))]  # Name of all joints
        JointIndicesComplete = [i for i in range(0, len(bulletJointNames))]
        self.controlledJoints = [i for (i, n) in enumerate(bulletJointNames) if n not in nonControlledJoints]
        '''if False:
            print("===================")
            print("List of all joints - ",len(bulletJointNames)," joints :")
            for i in JointIndicesComplete:
                print(i," - ",bulletJointNames[i])
                #print(i," - ",p.getJointInfo(self.robotId, i)[1],"\n   vel ",p.getJointInfo(self.robotId, i)[11])
            print("===================")
            print("===================")
            print("List of joints controlled - ",len(self.controlledJoints)," joints :")
            listVel = []
            for i in self.controlledJoints:
                print(i," - ",bulletJointNames[i])
                listVel.append(p.getJointInfo(self.robotId, i)[11])
            print("mapJointVel = ",listVel)
            print("===================")  
            '''
        # GAINS
        self.gainsP = np.array([
            200.,  200.,                                        # TORSO 0-1
            6.0, 6.0,                                           # HEAD 2-3
            200.0, 400.,  100.,  100.,  10.,  10.,  10.,   # LEFT ARM 4-10
            200.,  400.,  100.,  100.,  10.,  10.,  10.,   # RIGHT ARM 11-17
            100.,  100.,  100.,  100.,  100.,  100.,          # LEFT LEG 18-23
            100.,  100.,  100.,  100.,  100.,  100.,          # RIGHT LEG 24-29
        ])

        self.gainsD = gainsD = np.array([
            10.,  10.,                          # TORSO 0-1
            0.1, 0.1,                           # HEAD 2-3
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # LEFT ARM 4-10
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RIGHT ARM 11-17
            20.,  20.,  20.,  20.,  20.,  20.,  # LEFT LEG 18-23
            20.,  20.,  20.,  20.,  20.,  20.,  # RIGHT LEG 24-29
        ])

        self.controlledJointBounds  = []
        for i in self.controlledJointBounds:
            info = p.getJointInfo(self.robotId, i)
            self.controlledJointBounds.append(info[8],info[9])
        self.rmodel = [p.getJointState(self.robotId, i ) for i in self.controlledJoints]
        p.setJointMotorControlArray(self.robotId, jointIndices = self.controlledJoints, controlMode = p.VELOCITY_CONTROL, 
                                    targetVelocities = [0.0 for m in self.controlledJoints], forces = [0.0 for m in self.controlledJoints])

        self.jointTorques = [0.0 for m in self.controlledJoints]
        p.setJointMotorControlArray (self.robotId, jointIndices = self.controlledJoints, controlMode = p.TORQUE_CONTROL, forces = self.jointTorques)
        # End of initial
        #print(self.get_observation())
        #print("Initialization of Talos done")
        pass

    # help functions:
    def m2a(m): return np.array(m.flat)

    def a2m(a): return np.matrix(a).T

    def getCurrentState(self):
        # print("JointIndices:",len(jointIndices)," -> ",jointIndices)
        jointStates = p.getJointStates(self.robotId, self.controlledJoints)  # State of all joints
        jointPositions = np.array([value[0] for value in jointStates])
        jointVelocities = np.array([value[1] for value in jointStates])
        return jointPositions, jointVelocities

    def pdController(self, qdes, vdes):
        # Retrieve angular position and velocity of actuators
        # jointStates = pyb.getJointStates(robotId, controlledJoints)
        self.qmes, self.vmes = self.getCurrentState()
        # print("len qmes:",len(qmes)," and qdes:",len(qdes))
        # print("len vmes:",len(vmes)," and vdes:",len(vdes))
        # Torque PD controller
        self.jointTorques = self.gainsP * (qdes - self.qmes) + self.gainsD * (vdes - self.vmes)
        return self.jointTorques

    def get_ids(self):
        return self.client, self.robotId

    def apply_action(self, action):
        #Calculate Action state from normalization
        action_Real = []
        for i in range (30):
            joint_Force = action [i] * (self.mapJointLimit[i][1]-self.mapJointLimit[i][0]) + self.mapJointLimit[i][0]
            action_Real. append(joint_Force)
        vdes = [0.0 for _ in self.controlledJoints]
        jointPosition, jointVelocities = self.getCurrentState()
        qdes = action_Real
        torques = self.pdController(qdes, vdes)
        p.setJointMotorControlArray(self.robotId, self.controlledJoints, controlMode = p.TORQUE_CONTROL, forces = torques)



    def get_observation(self):
        observation = []
        #print("Controlled joints ",len(self.controlledJoints)," : ",self.controlledJoints)
        jointStates = p.getJointStates(self.robotId, self.controlledJoints)
        jointPositions = np.array([ value[0] for value in jointStates ])
        jointVelocities = np.array([ value[1] for value in jointStates ])
        #print("Len : ",len(jointPositions)," and ",len(jointVelocities))
        #print("position:",jointPositions)
        #print("velocities:", jointVelocities)
        obs_Pos, obs_Vel = self.normalizeJoints(jointPositions, jointVelocities)
        #observation.append(joint_State[:2])
        #observation = np.array(observation)
        #observation[0]= obs_Pos
        #observation[1]= obs_Vel
        observation = np.append(obs_Pos,obs_Vel)
        #observation = np.array(observation)
        #print("Normalized:", observation)
        #observation.append(obs_Vel)
        return observation

    def normalizeJoints(self, jointPositions, jointVelocities):
        jointPositionsNormalized, jointVelocitiesNormalized = jointPositions[::], jointVelocities[::]
        # Normalize joint positions
        #a = jointPositions
        for i in range (len(jointVelocities)):
            joint_Pos = (jointPositions[i] - self.mapJointLimit[i][0])/(self.mapJointLimit[i][1]-self.mapJointLimit[i][0])
            jointPositionsNormalized[i]= joint_Pos
        obs_position = np.array(jointPositions)
        # Normalize joint velocities
        # TO DO
        for j in range (len(jointVelocities)):
            joint_Vel = (jointVelocities[j]-(-self.mapJointVel[i]))/(2*self.mapJointVel[j])
            jointVelocitiesNormalized[j] = joint_Vel
        obs_Velocities = np.array(jointVelocitiesNormalized)
        return obs_position, obs_Velocities
        #return jointPositionsNormalized,jointVelocitiesNormalized
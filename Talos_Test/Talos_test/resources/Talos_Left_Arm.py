import pybullet as p
import numpy as np
import os
import pybullet_data
import example_robot_data

class Talos:
    def __init__(self,client):
        self.client = client

        #basic setup
        self.robotStartPosition = [0.0, 0.0, 1.08]  # 1.08]
        self.robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        #load Talos
        #p.setTimeStep(1e-3)
        self.robotId = p.loadURDF("talos_reduced.urdf",self.robotStartPosition, self.robotStartOrientation, useFixedBase = True)

        #Joint Limit
        self.mapJointLimit = [
            [-1.57079632679, 0.523598775598],   # LEFT ARM 4-10
            [0.0, 2.87979326579],
            [-2.44346095279, 2.44346095279],
            [-2.35619449019, 0.0],
            [-2.53072741539, 2.53072741539],
            [-1.3962634016, 1.3962634016],
            [-0.698131700798, 0.698131700798],
        ]

        self.mapJointVel =  [2.7, 3.66, 4.58, 4.58, 1.95, 1.76, 1.76]

        nonControlledJoints = [
            "torso_1_joint",# Other Body Joints
            "torso_2_joint",
            "head_1_joint",
            "head_2_joint",

            "arm_right_1_joint",#Rgiht Joints
            "arm_right_2_joint",
            "arm_right_3_joint",
            "arm_right_4_joint",
            "arm_right_5_joint",
            "arm_right_6_joint",
            "arm_right_7_joint",

            "leg_left_1_joint", #Left Leg
            "leg_left_2_joint",
            "leg_left_3_joint",
            "leg_left_4_joint",
            "leg_left_5_joint",
            "leg_left_6_joint",

            "leg_right_1_joint", #right Leg
            "leg_right_2_joint",
            "leg_right_3_joint",
            "leg_right_4_joint",
            "leg_right_5_joint",
            "leg_right_6_joint",

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
            #200.,  200.,                                        # TORSO 0-1
            #6.0, 6.0,                                           # HEAD 2-3
            200.0, 400.,  100.,  100.,  10.,  10.,  10.,   # LEFT ARM 4-10
            #200.,  400.,  100.,  100.,  10.,  10.,  10.,   # RIGHT ARM 11-17
            #100.,  100.,  100.,  100.,  100.,  100.,          # LEFT LEG 18-23
            #100.,  100.,  100.,  100.,  100.,  100.,          # RIGHT LEG 24-29
        ])

        self.gainsD = gainsD = np.array([
            #10.,  10.,                          # TORSO 0-1
            #0.1, 0.1,                           # HEAD 2-3
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # LEFT ARM 4-10
            #0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # RIGHT ARM 11-17
            #20.,  20.,  20.,  20.,  20.,  20.,  # LEFT LEG 18-23
            #20.,  20.,  20.,  20.,  20.,  20.,  # RIGHT LEG 24-29
        ])

        self.controlledJointBounds = self.mapJointLimit
        #for i in self.controlledJoints:
        #    info = p.getJointInfo(self.robotId, i)
        #    print(info[8],info[9])
        #    self.controlledJointBounds.append(info[8])
        #    self.controlledJointBounds.append(info[9])
        self.rmodel = [p.getJointState(self.robotId, i ) for i in self.controlledJoints]
        p.setJointMotorControlArray(self.robotId, jointIndices = self.controlledJoints, controlMode = p.VELOCITY_CONTROL, 
                                    targetVelocities = [0.0 for m in self.controlledJoints], forces = [0.0 for m in self.controlledJoints])

        self.jointTorques = [0.0 for m in self.controlledJoints]
        p.setJointMotorControlArray (self.robotId, jointIndices = self.controlledJoints, controlMode = p.TORQUE_CONTROL, forces = self.jointTorques)
        # End of initial
        #print(self.get_observation())
        #print("Initialization of Talos done")
        #pass

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

    def applyAction(self, joints_Torques):
        #Calculate Action state from normalization
        #action_Real = []
        #for i in range (len(action)):
        #    joint_Force = action [i] * (self.mapJointLimit[i][1]-self.mapJointLimit[i][0]) + self.mapJointLimit[i][0]
        #    action_Real. append(joint_Force)
        #vdes = [0.0 for _ in self.controlledJoints]
        jointPosition, jointVelocities = self.getCurrentState()
        #qdes = action
        #torques = self.pdController(qdes, vdes)
        p.setJointMotorControlArray(self.robotId, self.controlledJoints, controlMode = p.TORQUE_CONTROL, forces = joints_Torques)


    def getJointsValues(self):
        jointStates = p.getJointStates(self.robotId, self.controlledJoints)
        jointPositions = np.array([ value[0] for value in jointStates ])
        jointVelocities = np.array([ value[1] for value in jointStates ])
        return jointPositions, jointVelocities

    def get_observation(self):
        observation = []
        jointPositions, jointVelocities = self.getJointsValues()
        observation = np.append(jointPositions,jointVelocities)
        print (observation)
        return observation

    def get_observation_normalized(self):
        jointPositions, jointVelocities = self.getJointsValues()
        # Normalize joints position and velocity
        obs_Pos, obs_Vel = self.normalizeJoints(jointPositions, jointVelocities)
        observation = np.append(obs_Pos,obs_Vel)
        return observation

    def normalizeJoints(self, jointPositions, jointVelocities):
        jointPositionsNormalized, jointVelocitiesNormalized = jointPositions[::], jointVelocities[::]
        # Normalize joint positions
        for i in range (len(jointVelocities)):
            joint_Pos = (jointPositions[i] - self.mapJointLimit[i][0])/(self.mapJointLimit[i][1]-self.mapJointLimit[i][0])
            jointPositionsNormalized[i]= joint_Pos
        obs_position = np.array(jointPositions)
        # Normalize joint velocities
        for j in range (len(jointVelocities)):
            jointVelocitiesNormalized[j] = jointVelocities[j] / self.mapJointVel[j]
        obs_Velocities = np.array(jointVelocitiesNormalized)
        return obs_position, obs_Velocities
# Talos
 Here is the source code for RL trainning for Talos robot.   
 This project are build under Python 3.63 and Pybullet environment. Tested on Windows 10 and Linux (testing).   
 The following instruction would lead you to created an environment to run this project succesfully. 
 
 ## Patch News V-0.1:
 The robot are aim to raise his left arm during the trainning in this vision.  
 The basic function of Talos are tested. 
 
 ### Unsolved Problems and Next Stage:
 Learning to Stand.   
 Talos's joints was not returned to origin position sometimes after each episode (Obsevered by GUI.)  
 Issues about negative-log in tensorboard.   
 
 ### Solved Problems:  
 PID controlled.   
 Identidy the Controlled Joints.   
 Basic environment set-up and tested.   
 
 
 ## Instruction:
 Here is related API and packages use in this project. 
 ### Python vision:
 Python environment <=3.70 are required to run this project.   
 Python 3.6.13 are used to build the whole project. 
 
 ### Pybullet:
 To install Pybullet use following code in anaconda(Windows) and command line (Linux):  
 `pip install pybullet`  
 For more information on PyBullet, read look [PyBullet ](https://pybullet.org/wordpress/).
 
 ### Open Ai Gym:
 To install Gym use following code in anaconda(Windows) and command line (Linux):
 `pip install gym`
 

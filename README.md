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
 For more information on PyBullet, read [PyBullet ](https://pybullet.org/wordpress/).
 
 ### Open Ai Gym:
 To install Gym use following code in anaconda(Windows) and command line (Linux):   
 `pip install gym`  
 For more information on PyBullet, read [Gym offical installation documentation](https://gym.openai.com/docs/#installation).

### Pytorch:
To install Torch use following code in anaconda(Windows) and command line (Linux):  
`pip install torch`  
For more information in Pytorch, read [Pytorch offical cite](https://pytorch.org/).

### Example_robot_data:
To install example_robot_data use following code in anaconda(Windows) :    
`conda install example-robot-data -c conda-forge`  
For Linux Users: 
Please following instruction on [Gepetto's page](https://github.com/Gepetto/example-robot-data).

### Stable-baselines:
To install Torch use following code in anaconda(Windows) and command line (Linux):  
`pip install stable-baselines`  
For more information in Stable-baselines, read [Stable-Baseline](https://github.com/hill-a/stable-baselines).

### Tensorflow 
**Notification: Tensorflow >= 2.0 are not support for stable-baselines. This project are using TF 1.15.0 for build.  
Python >= 3.8 require Tensorflow 2.2 or higher to run**  
To install Tensorflow use following code in anaconda(Windows):     
`conda install tensorflow==1.15`  
Required Python 3.6 or 3.7    
To install Tensorflow on Linux:  
`pip install tensorflow==1.15`  
If you need more helps on Tensorflow, Please check [Pypi for Tensorflow](https://pypi.org/project/tensorflow/) and [Tensorflow](https://www.tensorflow.org/install/). 

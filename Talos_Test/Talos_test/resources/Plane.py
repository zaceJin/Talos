import pybullet as p
import os
import pybullet_data

class Plane:
    def __init__(self,client):
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")  # Load horizontal plane
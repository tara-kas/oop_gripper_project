import pybullet as p
import pybullet_data
import time
import math
from SceneObject import SceneObject, Cube
from Gripper import Gripper, TwoFingerGripper
import os

if __name__=="__main__":
    
    # load in "cube_small.urdf" from objects folder using its path
    cube_path = os.path.join(os.path.dirname(__file__), "objects", "cube_small.urdf")
    print(cube_path)
    
    # setup the environment
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    # p.addUserDebugLine((0,0,0)) finish adding the params
    
    # uncomment to change camera angle:
    # p.resetDebugVisualizerCamera(
    # cameraDistance = 1.5,
    # cameraYaw = 50,
    # cameraPitch = -35,
    # cameraTargetPosition = [0,0,0]
    # )

    planeID = p.loadURDF("plane.urdf")
    
    boxID = Cube(cube_path,(0,0,-10))
    
    gripper1 = TwoFingerGripper((1,0,1),(0,math.pi/4,math.pi))
    gripper1.load()
    # time.sleep(3)
    gripper1.attach_fixed(0)
    gripper1.teleport((0.3,0,0.3))
    time.sleep(5)
    gripper1.move(1,1,1)

    
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
import pybullet as p
import pybullet_data
import time
import math
from SceneObject import SceneObject, Cube, Cylinder
from Gripper import Gripper, TwoFingerGripper, ThreeFingerGripper
import os

if __name__=="__main__":
    
    # load in "cube_small.urdf" from objects folder using its path
    cube_path = os.path.join(os.path.dirname(__file__), "objects", "cube_small.urdf")
    cyl_path = os.path.join(os.path.dirname(__file__), "objects", "cylinder.urdf")
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
    
    # boxID = Cube(cube_path,(0.6,0.3,0.02))
    cylID = Cylinder(cyl_path,(0,0,0.02))
    
    # gripper1 = TwoFingerGripper((0, 0, 0.02), (0, 0, 0)) 
    gripper2 = ThreeFingerGripper(position=(0,0,0.5), orientation=(math.pi,0,0))
    gripper2.load()
    gripper2.start()
    # gripper2.attach_fixed(offset=[0,0,0])
    gripper2.grasp_and_lift(cylID)

    
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
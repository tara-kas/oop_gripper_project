import pybullet as p
import pybullet_data
import time
import math
from SceneObject import SceneObject, Cube, Cylinder
from Gripper import Gripper, TwoFingerGripper, ThreeFingerGripper
import os
import numpy as np

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

    n = 5  # no. of iterations
    grippers = {}   # initialise the grippers
    
    planeID = p.loadURDF("plane.urdf")
    
    boxID = Cube(cube_path,(0,0,0.05))  # z is half the height of the cube
 
    xyz = tuple(Gripper.get_random_start_position())  # convert numpy array to tuple
    # rpy = (0, 0, 0)   
    rpy = Gripper.orient_towards_origin(xyz, random_roll=False)
    
    gripper1 = TwoFingerGripper(xyz, rpy)
    
    # OBJECT 1: CUBE
    for i in range(n):
        xyz = Gripper.get_random_start_position()
        rpy = Gripper.orient_towards_origin(xyz)
        xyz.tolist()
        gripper = TwoFingerGripper(xyz,rpy)
        print(xyz)
        print(rpy)
        gripper.load()
        gripper.teleport(obj_id=boxID.id, offset=boxID.grasp_offset)
        print(f"trial {i+1} w/ cube")
        # time.sleep(1)
        # open gripper
        gripper.grasp_and_lift(boxID)
        print(f"{gripper.position} after grasp and lift")
       
        # ADD LOGIC result (success or failure) into dict
        grippers[gripper] = None
        
         # reset cube to origin after gripper is done
        boxID.position = (0, 0, 0.05)
        p.resetBasePositionAndOrientation(boxID.id, boxID.position, boxID.orientation)
        
        p.removeBody(gripper.id)

        # move closer a bit
        # close gripper
        # move away
    
    p.removeBody(boxID.id)
        
    # OBJECT 2: CLYINDER
    cylID = Cylinder(cyl_path,(0,0,0.02))
    
    for i in range(n):
        xyz = Gripper.get_random_start_position()
        rpy = Gripper.orient_towards_origin(xyz)
        xyz.tolist()
        gripper = TwoFingerGripper(xyz,rpy)
        print(xyz)
        print(rpy)
        gripper.load()
        gripper.teleport(obj_id=cylID.id, offset=cylID.grasp_offset)
        print(f"trial {i+1} w/ cylinder")
        # time.sleep(1)
        # open gripper
        gripper.grasp_and_lift(cylID)
        
        # ADD LOGIC result (success or failure) into dict
        grippers[gripper] = None
        
        # reset cylinder to origin after gripper is done
        cylID.position = (0, 0, 0.02)
        p.resetBasePositionAndOrientation(cylID.id, cylID.position, cylID.orientation)
        
        p.removeBody(gripper.id)
        
    p.removeBody(cylID.id)

    print(grippers)

    
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
    
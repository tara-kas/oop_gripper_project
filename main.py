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
    
    # # boxID = Cube(cube_path,(0.6,0.3,0.02))
    cylID = Cylinder(cyl_path,(0,0,0.02))
    obj_pos = cylID.position
    # xyz = Gripper.get_random_start_position()
    xyz = tuple(Gripper.get_random_start_position())  # convert numpy array to tuple
    # rpy = (0, 0, 0)   
    rpy = Gripper.orient_towards_origin(xyz, random_roll=False)
    
    gripper1 = TwoFingerGripper(xyz, rpy)

    
    # gripper1 = TwoFingerGripper((0, 0, 0.02), (0, 0, 0)) 
    # gripper1 = ThreeFingerGripper(position=(0,0,0.5), orientation=(math.pi,0,0))
    gripper1.load()
    gripper1.start()
    gripper1.grasp_and_lift(cylID)
    gripper1.attach_fixed(target_id=cylID.id)

    # n = 5 
    # grippers = {} 
    # for i in range(n):
    #     xyz = Gripper.get_random_start_position()
    #     rpy = Gripper.orient_towards_origin(xyz)
    #     xyz.tolist()
    #     grippers[i] = TwoFingerGripper(xyz,rpy)
    #     print(xyz)
    #     print(rpy)
    #     grippers[i].load()
    #     grippers[i].start()
    #     grippers[i].attach_fixed(offset=[0,0,0])
        
    # grippers[0].grasp_and_lift(cylID)
    # print(grippers)
    # gripper1 = TwoFingerGripper((1,0,1),(0,math.pi/4,math.pi))
    # gripper1.load()
    # # time.sleep(3)
    # gripper1.attach_fixed(0)
    # gripper1.teleport((0.3,0,0.3))
    # time.sleep(5)
    # gripper1.move(1,1,1)
    
    for i in range(n):
        xyz = Gripper.get_random_start_position()
        rpy = Gripper.orient_towards_origin(xyz)
        xyz.tolist()
        grippers[i] = TwoFingerGripper(xyz,rpy)
        print(xyz)
        print(rpy)
        grippers[i].load()
        grippers[i].teleport(obj_id=boxID.id)
        # time.sleep(3)
        # open gripper
        # move closer a bit
        # close gripper
        # move away

    print(grippers)

    
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
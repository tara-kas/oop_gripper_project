import pybullet as p
import pybullet_data
import time
from SceneObject import SceneObject, Cube
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
    
    # uncomment to change camera angle:
    # p.resetDebugVisualizerCamera(
    # cameraDistance = 1.5,
    # cameraYaw = 50,
    # cameraPitch = -35,
    # cameraTargetPosition = [0,0,0]
    # )

    planeID = p.loadURDF("plane.urdf")
    
    boxID = Cube(cube_path,(0,0,0.26))
    
    for i in range(1000):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
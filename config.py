"""configuration settings and environment setup"""
import os
import pybullet as p
import pybullet_data

# paths
BASE_PATH = os.path.dirname(__file__)
CUBE_URDF = os.path.join(BASE_PATH, "objects", "cube_small.urdf")
DUCK_URDF = os.path.join(BASE_PATH, "objects", "duck_vhacd.urdf")

# for data collection
SAMPLES_PER_COMBINATION = 250
RADIUS = 2                     # distance from object for starting position
USE_GUI = False                # whether to use GUI mode

# output path for clf
MODEL_PATHS = [os.path.join(BASE_PATH, "grasp_classifier_two_cube.pkl"),
              os.path.join(BASE_PATH, "grasp_classifier_two_duck.pkl"),
              os.path.join(BASE_PATH, "grasp_classifier_three_cube.pkl"),
              os.path.join(BASE_PATH, "grasp_classifier_three_duck.pkl")]

def setup_environment(gui=None):
    """
    initialize PyBullet environment
    
    args:
        gui: If True, use GUI mode; if None, uses DIRECT mode (headless)
        
    returns:
        physics_client: PyBullet physics client ID
    """ 
    if gui:
        physics_client = p.connect(p.GUI)
    else:
        physics_client = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    
    # load ground plane
    p.loadURDF("plane.urdf")
    
    # set camera position
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.1]
    )
    
    return physics_client
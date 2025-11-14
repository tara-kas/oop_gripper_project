import pybullet as p
import pybullet_data
import random
import time
from abc import abstractmethod, ABC

class SceneObject:
    """ base class for any object that will be in the pybullet scene """
    
    def __init__(self, urdf, position=(0,0,0), orientation=(0,0,0)):
        self.urdf = urdf
        self.position = position
        self.orientation = p.getQuaternionFromEuler(orientation)
        self.id = None      # will be modified in function load()
        self.name = None    # will be modified in function update_name()
        
    def load(self):
        """ load object into scene """
        self.id = p.loadURDF(self.urdf, self.position, self.orientation, useFixedBase=True)
        return self.id

    
    def update_name(self, new_name="default_name"):
        """ initialise or update name of object """
        self.name = new_name
        
class Cube(SceneObject):
    """ object to be grabbed by the gripper """
    
    def __init__(self, urdf, position=(0,0,0), orientation=(0,0,0), scale=0.8):
        super().__init__(urdf, position=position, orientation=orientation)  
        self.scale = scale  
        self.id = p.loadURDF(self.urdf, self.position, self.orientation, globalScaling=self.scale)
        self.name = super().update_name("Box")  
        aabb_min, aabb_max = p.getAABB(self.id)
        self.grasp_height = aabb_max[2]  
        self.grasp_offset = 0.13    # half of the diagonal + a small margin

class Cylinder(SceneObject):
    """Cylinder object to be grabbed by the gripper, with optional scaling."""
    
    def __init__(self, urdf, position=(0, 0, 0), orientation=(0, 0, 0), scale=0.8):
        super().__init__(urdf, position=position, orientation=orientation)
        
        self.scale = scale
        self.id = p.loadURDF(
            self.urdf, 
            self.position, 
            self.orientation, 
            globalScaling=self.scale
        )
    
        self.name = super().update_name("Cylinder")
        
        aabb_min, aabb_max = p.getAABB(self.id)
        self.grasp_height = aabb_max[2] + 0.01 
        self.grasp_offset = 0.12    # radius + small margin

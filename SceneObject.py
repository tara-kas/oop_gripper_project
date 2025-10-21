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
        self.id = p.loadURDF(self.urdf, self.position, self.orientation)
        return self.id
    
    def update_name(self, new_name="default_name"):
        """ initialise or update name of object """
        self.name = new_name
        
class Cube(SceneObject):
    """ object to be grabbed by the gripper """
    
    def __init__(self, urdf, position=(0,0,0), orientation=(0,0,0)):
        super().__init__(urdf, position=(0,0,0), orientation=(0,0,0))
        self.id = super().load()                # load in object
        self.name = super().update_name("Box")  # give name to object
        
    @property    
    @abstractmethod
    def get_position(self):     # get position in pybullet worldframe
        pass
import pybullet as p
import pybullet_data
import time
from abc import ABC, abstractmethod
from SceneObject import SceneObject
# need to import a gripper urdf

class Gripper(ABC, SceneObject):
    """ general gripper class for grasping objects """
    def __init__(self, urdf, position=(0,0,0), orientation=(0,0,0)):
        super().__init__(urdf, position=(0,0,0), orientation=(0,0,0))
        self.name = super().update_name("Gripper")  # give name to object

        self.constraint_id = None
        self.grasp_moving = False
        
    # inherits load() function from parent
    
    def attach_fixed(self, offset):
        """ attach gripper to a fixed world position """
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.id,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=offset,
            childFramePosition=self.base_position)
    
    def move(self, z, yaw=0.0):
        pass

    @abstractmethod                                         # all this implementation in child classes
    def open(self):
        pass
    @abstractmethod
    def close(self):
       pass
    @abstractmethod
    def maintain_grip(self):
        pass

    def get_gripper_pose_relative_to_object(self, obj):
        #   Return 6D pose features relative to object
        pass

    def execute_grasp_test(self, obj, grasp_pose, lift_height=0.3):
        #   Execute full grasp sequence and return success
        pass

    def is_grasp_successful(self, obj, initial_pos, hold_time=3.0):
        # Check if object remains grasped
        pass

class TwoFingerGripper(Gripper):
    def __init__(self, position=(0.5, 0.3, 0.7)):
        super().__init__("pr2_gripper.urdf", position)
        pass

    def open(self):
        #  Open gripper fingers
        pass

    def close(self):
        # Close gripper fingers
        pass

    def maintain_grip(self):
        # Maintain grip force
        pass
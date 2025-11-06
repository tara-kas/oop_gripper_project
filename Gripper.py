import pybullet as p
import pybullet_data
import time
import math
from abc import ABC, abstractmethod
from SceneObject import SceneObject
import numpy as np
# need to import a gripper urdf

class Gripper(ABC, SceneObject):
    """ general gripper class for grasping objects """
    def __init__(self, urdf, position=(0,0,0), orientation=(0,0,0)):
        super().__init__(urdf, position, orientation)
        self.name = super().update_name("Gripper")  # give name to object
        
        self.roll, self.pitch, self.yaw = orientation

        self.constraint_id = None
        self.grasp_moving = False
        
    # inherits load() function from parent
    
    @property
    def update_orientation(self):
        return self.roll, self.pitch, self.yaw
    
    @update_orientation.setter
    def update_orientation(self,orientation):
        self.roll = orientation[0]
        self.pitch = orientation[1]
        self.yaw = orientation[2]
    
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
            childFramePosition=self.position)
    
    def teleport(self, move_to=(0,0,0), steps=100):
        self.position = move_to 
        p.resetBasePositionAndOrientation(self.id, self.position, self.orientation)
        print(f"{self.name} moved to {self.position}.")
    
    def move(self, x,y,z, roll=None, pitch=None, yaw=None):
        """Move gripper to a new position and orientation."""
        print(f"moving gripper to {x},{y},{z}")
        
        # if roll, pitch, or yaw aren't given, default to what is already was
        roll = self.roll if roll is None else roll
        pitch = self.pitch if pitch is None else pitch
        yaw = self.yaw if yaw is None else yaw
        
        if self.constraint_id is None:
            raise ValueError("Gripper must be fixed before moving.")
        p.changeConstraint(
            self.constraint_id,
            jointChildPivot=[x,y,z],
            jointChildFrameOrientation=p.getQuaternionFromEuler([roll, pitch, yaw]),
            maxForce=20
        )
        # p.changeDynamics(self.id, -1, mass=0.00001)         # make mass super small so less inertia and momentum

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
        #   rot_mateturn 6D pose features relative to object
        pass

    def execute_grasp_test(self, obj, grasp_pose, lift_height=0.3):
        #   Execute full grasp sequence and return success
        pass

    def is_grasp_successful(self, obj, initial_pos, hold_time=3.0):
        # Check if object remains grasped
        pass
    
    def get_random_start_position(radius=2):
        """initialise gripper at a random position, distance 1 away from the object at origin (0,0,0) 
        """
        # generate random angles
        theta = np.random.uniform(0,2*np.pi)
        phi = np.random.uniform(0,np.pi/2)    # limit it to top hemisphere (above the plane)
        
        # sphere into cartesian coords
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)
        
        return np.array([x,y,z])
    
    # def orient_towards_origin(pos):
    #     """compute quarternion given position in (x,y,z) to point towards origin from +Z"""
    #     # get forward direction
    #     forward = -pos/np.linalg.norm(pos)
        
    #     # make a temp up vector
    #     up = np.array([0,0,1])
    #     # check in case too z-axis too parallel for cross product
    #     if abs(np.dot(forward,up)) > 0.99:
    #         up = np.array([0,1,0])
            
    #     # make right and up vectors
    #     right = np.cross(up,forward)
    #     right /= np.linalg.norm(right)
    #     up = np.cross(forward, right)       # rewrite up as the correct vector
        
    #     rot_mat = np.vstack([right,up,forward])
        
        # # change rotation matrix (3x3) into euler angle form (roll, pitch, yaw)
        # sy = np.sqrt(rot_mat[0,0]**2 + rot_mat[1,0]**2)
        # singular = sy < 1e-6

        # if not singular:
        #     roll = np.arctan2(rot_mat[2,1], rot_mat[2,2])
        #     pitch = np.arctan2(-rot_mat[2,0], sy)
        #     yaw = np.arctan2(rot_mat[1,0], rot_mat[0,0])
        # else:
        #     # in case of gimbal lock
        #     roll = np.arctan2(-rot_mat[1,2], rot_mat[1,1])
        #     pitch = np.arctan2(-rot_mat[2,0], sy)
        #     yaw = 0
        
        # return (roll,pitch,yaw)
    
    def orient_towards_origin(pos):
        x, y, z = pos
        
        # yaw: rotate around Z toward origin
        yaw = np.arctan2(-y, -x)

        # pitch: tilt down/up to point at origin
        distance_xy = np.sqrt(x**2 + y**2)
        pitch = np.arctan2(z, distance_xy)

        # roll: random for variability
        roll = np.random.uniform(-np.pi, np.pi)  # random roll

        return (roll, pitch, yaw)


class TwoFingerGripper(Gripper):
    def __init__(self, position=(0, 0, 0), orientation=(0,0,0)):
        super().__init__("pr2_gripper.urdf", position, orientation)
        self.name = "TwoFingerGripper"

    def open(self):
        #  Open gripper fingers
        pass

    def close(self):
        # Close gripper fingers
        pass

    def maintain_grip(self):
        # Maintain grip force
        pass
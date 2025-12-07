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
    
    @property
    def update_orientation(self):
        return self.roll, self.pitch, self.yaw
    
    @update_orientation.setter
    def update_orientation(self,orientation):
        self.roll = orientation[0]
        self.pitch = orientation[1]
        self.yaw = orientation[2]

    def attach_fixed(self, target_id):
    # Get the world poses
        gripper_pos, gripper_orn = p.getBasePositionAndOrientation(self.id)
        target_pos, target_orn = p.getBasePositionAndOrientation(target_id)

        # Compute relative transform: where gripper is relative to target
        inv_target_pos, inv_target_orn = p.invertTransform(target_pos, target_orn)
        local_pos, local_orn = p.multiplyTransforms(inv_target_pos, inv_target_orn, gripper_pos, gripper_orn)

        # Create fixed constraint preserving that relative offset
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=target_id,
            parentLinkIndex=-1,
            childBodyUniqueId=self.id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=local_pos,
            parentFrameOrientation=local_orn,
            childFramePosition=[0, 0, 0],
            childFrameOrientation=[0, 0, 0, 1]
        )
    
    def teleport(self,obj_id=None, move_to=None, steps=100, offset=0.12):
        if move_to:
            # create a copy of the position to avoid shared references
            self.position = np.array(move_to).copy()
            p.resetBasePositionAndOrientation(self.id, self.position, self.orientation)
            print(f"{self.name} teleported directly to {self.position}.")
            return
            
        elif obj_id:
            # get object position
            obj_pos, _ = p.getBasePositionAndOrientation(obj_id)
            obj_pos = np.array(obj_pos)
            
            # make gripper position into numpy array
            current_pos = np.array(self.position)

            # get direction vector
            direction = obj_pos - current_pos
            direction_norm = np.linalg.norm(direction)
            
            if direction_norm < 1e-6:
                raise ValueError("object and gripper have same position")
            
            direction /= direction_norm         # normalise vector
            
            orientation_euler = self.orient_towards_origin(direction)
            self.roll, self.pitch, self.yaw = orientation_euler
            quat = p.getQuaternionFromEuler(orientation_euler)
            self.orientation = quat
            
            self.position = obj_pos - direction * offset
                
            p.resetBasePositionAndOrientation(self.id, self.position, self.orientation)
            print(f"{self.name} moved to {self.position}.")
            return
    
        else:
            raise ValueError("You must specify either move_to or obj_id.")
    
    def move(self, z, x=None,y=None, roll=None, pitch=None, yaw=None):
        """Move gripper to a new position and orientation."""
        # print(f"moving gripper to {x},{y},{z}")

        cur_x, cur_y, _ = self.position
        if x is None:
            x = cur_x
        if y is None:
            y = cur_y
        self.position = (x, y, z)
        
        # if roll, pitch, or yaw aren't given, default to what is already was
        roll = self.roll if roll is None else roll
        pitch = self.pitch if pitch is None else pitch
        yaw = self.yaw if yaw is None else yaw
        
        # if self.constraint_id is None:
        #     raise ValueError("Gripper must be fixed before moving.")
        
        p.changeConstraint(
            self.constraint_id,
            jointChildPivot=[x,y,z],
            jointChildFrameOrientation=p.getQuaternionFromEuler([roll, pitch, yaw]),
            maxForce=100
        )
        # p.changeDynamics(self.id, -1, mass=0.00001)         # make mass super small so less inertia and momentum
    @staticmethod
    def orient_towards_origin(pos, random_roll=False):
        x, y, z = pos
        
        # yaw: rotate around Z toward origin
        yaw = np.arctan2(-y, -x)

        # pitch: tilt down/up to point at origin
        distance_xy = np.sqrt(x**2 + y**2)
        pitch = np.arctan2(z, distance_xy)

        if random_roll:
            roll = np.random.uniform(-0.1, 0.1)  # small roll, max ±0.1 rad
        else:
                roll = 0.0

        return (roll, pitch, yaw)
    @staticmethod
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
    
    def compute_approach_pose(self, obj_pos, approach_distance=0.3, height_offset=0.05):
        x_o, y_o, z_o = obj_pos
        # pick direction vector from gripper to object
        yaw = np.arctan2(y_o, x_o)  # yaw that faces the object
        
        # approach position a bit away in front of object along that yaw
        x_g = x_o - approach_distance * np.cos(yaw)
        y_g = y_o - approach_distance * np.sin(yaw)
        z_g = z_o + height_offset
        
        return (x_g, y_g, z_g, yaw)

        
    @abstractmethod                                       
    def open(self):
        pass
    @abstractmethod
    def close(self):
       pass
    # @abstractmethod
    # def maintain_grip(self):
    #     pass

    def get_gripper_pose_relative_to_object(self, obj):
        #   rot_mateturn 6D pose features relative to object
        pass

    def execute_grasp_test(self, obj, grasp_pose, lift_height=0.3):
        #   Execute full grasp sequence and return success
        pass

    def is_grasp_successful(self, obj, initial_pos, hold_time=3.0):
        # Check if object remains grasped
        pass




class TwoFingerGripper(Gripper):
    def __init__(self, position=(0, 0, 0), orientation=(0,0,0)):
        super().__init__("pr2_gripper.urdf", position, orientation)
        self.name = "TwoFingerGripper"

    def start(self):
        """Initialize the gripper joints so it can open/close."""
        initial_positions = [0.550569, 0.0, 0.549657, 0.0]  
        for i, pos in enumerate(initial_positions):
            p.resetJointState(self.id, i, pos)

    def open(self, target=0.55):
        """Open gripper fingers to a wider position."""
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=target, maxVelocity=1, force=20)

    def close(self):
        """Close the gripper fingers."""
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=0.1, maxVelocity=1, force=20)

    def grasp_and_lift(self, obj, lift_height=0.4, lift_steps=150):
        """Perform grasping and lifting sequence with sustained gripping force."""
        yaw_angle = 0.0
        grasp_height = obj.grasp_height
        p.changeDynamics(obj.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        p.changeDynamics(self.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        
        # no need to move twice..?
        # self.teleport(obj_id=obj.id, offset=0.25)  # Increased offset for safety

        # turn gripper to face object
        self.attach_fixed(obj.id)
        
        # move forward towards object to grasp, but stop short to avoid pushing
        num_steps = 50
        current_pos = np.array(self.position)
        obj_pos = np.array(obj.position)
        
        # Calculate direction from gripper to object
        direction_to_obj = obj_pos - current_pos
        distance = np.linalg.norm(direction_to_obj)
        
        # Stop 0.08m before object center (adjust if needed)
        stop_distance = 0.1
        if distance > stop_distance:
            direction_to_obj = direction_to_obj / distance  # normalize
            target_pos = obj_pos - direction_to_obj * stop_distance
        else:
            target_pos = current_pos  # already close enough
        
        for i in range(num_steps):
            t = (i + 1) / num_steps
            x_step = current_pos[0] + (target_pos[0] - current_pos[0]) * t
            y_step = current_pos[1] + (target_pos[1] - current_pos[1]) * t
            z_step = current_pos[2] + (target_pos[2] - current_pos[2]) * t
            self.move(z=z_step, x=x_step, y=y_step)
            p.stepSimulation()
            time.sleep(1./240.)


        # --- Lower onto object ---
        # self.move(grasp_height, x=obj.position[0]-0.1, y=obj.position[1], yaw=yaw_angle)
        # print("\033[93mmove to the grasp height")
        # for _ in range(100):
        #     p.stepSimulation()
        #     time.sleep(1./240.)

        # --- Close gripper strongly ---
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=0.12, force=300, maxVelocity=2)
        for _ in range(200):  # allow contact to form
            p.stepSimulation()
            time.sleep(1./240.)

        # --- Continuous hold while lifting step-by-step ---
        z_current = grasp_height
        z_target = lift_height
        z_step = (z_target - z_current) / lift_steps

        for _ in range(lift_steps):
            z_current += z_step
            self.move(z_current, yaw=yaw_angle)

            # Continuously reapply strong grip to prevent slip
            for joint in [0, 2]:
                p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                        targetPosition=0.12, force=400, maxVelocity=2)

            p.stepSimulation()
            time.sleep(1./240.)




import os

class ThreeFingerGripper(Gripper):
    def __init__(self, position=(0, 0, 0), orientation=(0, 0, 0)):
        urdf_path = os.path.join(os.path.dirname(__file__), "objects", "sdh", "sdh.urdf")
        super().__init__(urdf_path, position, orientation)
        self.name = "ThreeFingerGripper"


    def start(self):
        self.num_joints = p.getNumJoints(self.id)
        self.joint_indices = list(range(self.num_joints))
        print(f"ThreeFingerGripper has {self.num_joints} joints")

        # set all to some relaxed initial position
        initial_positions = [-0.5 for _ in self.joint_indices]
        for i, pos in enumerate(initial_positions):
            p.resetJointState(self.id, i, pos)

    def open(self, target=0.5):
        """Open all fingers."""
        for j in self.joint_indices:
            p.setJointMotorControl2(
                self.id, j, p.POSITION_CONTROL,
                targetPosition=target, maxVelocity=1, force=20
            )

    def close(self, target=0.0):
        """Close all fingers."""
        for j in self.joint_indices:
            p.setJointMotorControl2(
                self.id, j, p.POSITION_CONTROL,
                targetPosition=target, maxVelocity=1, force=20
            )

    def grasp_and_lift(self, obj, lift_height=0.6, lift_steps=150):
        """Perform grasp and lift similar to the 2-finger gripper."""
        yaw_angle = 0.0 #  deleted 
        grasp_height = obj.grasp_height

        # Friction settings
        p.changeDynamics(obj.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        p.changeDynamics(self.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        
        # move gripper to object w/ offset
        self.teleport(obj_id=obj.id, offset=0.25)  # Increased offset for safety
        
        # Lower to grasp height
        self.move(grasp_height, x=obj.position[0], y=obj.position[1])
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)



        # Close the fingers to grasp
        for j in self.joint_indices:
            p.setJointMotorControl2(self.id, j, p.POSITION_CONTROL, targetPosition=0.1, force=300, maxVelocity=2)
        # for _ in range(100):
        #     p.stepSimulation()
        #     time.sleep(1./240.)

        # Lift gradually while keeping grip
        z_current = grasp_height
        z_target = lift_height
        z_step = (z_target - z_current) / lift_steps

        for _ in range(lift_steps):
            z_current += z_step
            self.move(z_current)
            for j in self.joint_indices:
                p.setJointMotorControl2(self.id, j, p.POSITION_CONTROL, targetPosition=0.1, force=400, maxVelocity=2)
            p.stepSimulation()
            time.sleep(1./240.)

    
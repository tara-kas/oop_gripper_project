import pybullet as p
import time
import os
import math
import numpy as np
from abc import ABC, abstractmethod


class Gripper(ABC):
    """Abstract base class for robotic grippers."""
    
    def __init__(self, urdf, position=(0, 0, 0), orientation=(0, 0, 0)):
        self.urdf = urdf
        self._position = np.array(position, dtype=float)
        self._orientation_euler = np.array(orientation, dtype=float)
        self._orientation_quat = p.getQuaternionFromEuler(orientation)
        self.id = None
        self.constraint_id = None
        self.name = "Gripper"
        
    @property
    def position(self):
        return tuple(self._position)
    
    @position.setter
    def position(self, value):
        self._position = np.array(value, dtype=float)
        
    @property
    def orientation_euler(self):
        return tuple(self._orientation_euler)
    
    @property
    def orientation_quat(self):
        return self._orientation_quat
    
    @orientation_quat.setter
    def orientation_quat(self, value):
        self._orientation_quat = value
        self._orientation_euler = np.array(p.getEulerFromQuaternion(value))
    
    def load(self):
        """Load gripper URDF into the scene."""
        self.id = p.loadURDF(self.urdf, self._position, self._orientation_quat)
        self._initialize_joints()
        return self.id
    
    @abstractmethod
    def _initialize_joints(self):
        """Initialize gripper joints to starting position."""
        pass
    
    @abstractmethod
    def open(self):
        """Open the gripper fingers."""
        pass
    
    @abstractmethod
    def close(self):
        """Close the gripper fingers."""
        pass
    
    @abstractmethod
    def get_finger_joints(self):
        """Return list of finger joint indices."""
        pass
    
    def create_constraint(self):
        """Create a fixed constraint to control gripper position."""
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.id,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=list(self._position),
            childFrameOrientation=self._orientation_quat
        )
        return self.constraint_id
    
    def move_to(self, position, orientation_euler=None, steps=10, step_delay=1./240.):
        """Move gripper to target position smoothly using constraint."""
        if self.constraint_id is None:
            self.create_constraint()
        
        if orientation_euler is None:
            orientation_euler = self._orientation_euler
        else:
            orientation_euler = np.array(orientation_euler)
            
        target_pos = np.array(position)
        start_pos = self._position.copy()
        start_orn = self._orientation_euler.copy()
        
        for i in range(steps):
            t = (i + 1) / steps
            interp_pos = start_pos + t * (target_pos - start_pos)
            interp_orn = start_orn + t * (orientation_euler - start_orn)
            
            p.changeConstraint(
                self.constraint_id,
                jointChildPivot=list(interp_pos),
                jointChildFrameOrientation=p.getQuaternionFromEuler(interp_orn),
                maxForce=300
            )
            p.stepSimulation()
            time.sleep(step_delay)
        
        self._position = target_pos
        self._orientation_euler = orientation_euler
        self._orientation_quat = p.getQuaternionFromEuler(orientation_euler)
    
    def teleport_to(self, position, orientation_euler=None):
        """Instantly move gripper to position (no simulation steps)."""
        self._position = np.array(position)
        if orientation_euler is not None:
            self._orientation_euler = np.array(orientation_euler)
            self._orientation_quat = p.getQuaternionFromEuler(orientation_euler)
        
        p.resetBasePositionAndOrientation(self.id, self._position, self._orientation_quat)
        
        if self.constraint_id is not None:
            p.changeConstraint(
                self.constraint_id,
                jointChildPivot=list(self._position),
                jointChildFrameOrientation=self._orientation_quat,
                maxForce=100
            )
    
    def grasp_and_lift(self, obj, approach_offset=0.15, lift_height=0.3, hold_time=1.0):
        """
        Execute full grasp sequence: approach, grasp, lift, hold, retract.
        Returns True if grasp is successful (object stays grasped).
        """
        # Store starting position for retraction
        start_pos = self._position.copy()
        start_orn = self._orientation_euler.copy()
        
        obj_pos, _ = p.getBasePositionAndOrientation(obj.id)
        obj_pos = np.array(obj_pos)
        
        # Set friction for better grasping
        p.changeDynamics(obj.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        for link in range(-1, p.getNumJoints(self.id)):
            p.changeDynamics(self.id, link, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        
        # Calculate approach direction (gripper to object)
        direction_to_obj = obj_pos - self._position
        direction_norm = np.linalg.norm(direction_to_obj)
        if direction_norm > 1e-6:
            direction_to_obj = direction_to_obj / direction_norm
        
        # Direction from object back towards gripper (for offsetting)
        direction_from_obj = -direction_to_obj
        
        # Approach pose: offset distance from object along approach direction
        approach_pos = obj_pos + direction_from_obj * approach_offset
        approach_pos[2] = max(approach_pos[2], obj.grasp_height + 0.02)
        
        # Grasp pose: stop short of object center to avoid pushing it
        # The gripper fingers will close around the object from this position
        grasp_offset_distance = 0.08  # distance to stop before object center
        grasp_pos = obj_pos + direction_from_obj * grasp_offset_distance
        grasp_pos[2] = obj.grasp_height
        
        # Step 1: Open gripper (fast)
        self.open()
        for _ in range(10):
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Step 2: Move to approach pose (fast)
        self.move_to(approach_pos, steps=25)
        
        # Step 3: Move to grasp pose (fast)
        self.move_to(grasp_pos, steps=20)
        
        # Step 4: Close gripper (fast)
        self.close()
        for _ in range(50):
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Step 5: Lift while maintaining grip (faster)
        lift_pos = grasp_pos.copy()
        lift_pos[2] = lift_height
        
        lift_steps = 50
        current_pos = self._position.copy()
        for i in range(lift_steps):
            t = (i + 1) / lift_steps
            interp_pos = current_pos + t * (lift_pos - current_pos)
            
            p.changeConstraint(
                self.constraint_id,
                jointChildPivot=list(interp_pos),
                jointChildFrameOrientation=self._orientation_quat,
                maxForce=300
            )
            
            self._apply_grip_force()
            p.stepSimulation()
            time.sleep(1./240.)
        
        self._position = lift_pos
        
        # Step 6: Hold and check success

        
        # Step 7: Retract back to starting position
        success = self._retract_to(start_pos, obj, start_orn, hold_time=3.0) 
        
        return success

    def _retract_to(self, position, obj, orientation_euler, hold_time=3.0, steps=40):
        """Retract gripper back to start position, maintaining grip, check success, then release."""
        target_pos = np.array(position)
        target_orn = np.array(orientation_euler)
        start_pos = self._position.copy()
        start_orn = self._orientation_euler.copy()
        
        # Move back to start position while maintaining grip
        for i in range(steps):
            t = (i + 1) / steps
            interp_pos = start_pos + t * (target_pos - start_pos)
            interp_orn = start_orn + t * (target_orn - start_orn)
            
            p.changeConstraint(
                self.constraint_id,
                jointChildPivot=list(interp_pos),
                jointChildFrameOrientation=p.getQuaternionFromEuler(interp_orn),
                maxForce=300
            )
            
            # Keep gripping during retraction
            self._apply_grip_force()
            
            p.stepSimulation()
            time.sleep(1./240.)
        
        self._position = target_pos
        self._orientation_euler = target_orn
        self._orientation_quat = p.getQuaternionFromEuler(target_orn)
        
        success = self._check_grasp_success(obj, hold_time)
        
        # Now release the object
        self.open()
        for _ in range(20):
            p.stepSimulation()
            time.sleep(1./240.)
            
        return success
    
    @abstractmethod
    def _apply_grip_force(self):
        """Apply continuous grip force during lifting."""
        pass
    
    def _check_grasp_success(self, obj, hold_time=3.0, max_distance=0.35):
        steps = int(hold_time * 240)
        
        for _ in range(steps):
            self._apply_grip_force()
            p.stepSimulation()
            
            # Get positions as numpy arrays immediately
            g_pos = np.array(p.getBasePositionAndOrientation(self.id)[0])
            o_pos = np.array(p.getBasePositionAndOrientation(obj.id)[0])
            
            # np.linalg.norm calculates Euclidean distance automatically
            # choose 3D in case the object flies up w/ the gripper but is let go
            if np.linalg.norm(g_pos - o_pos) > max_distance:
                print(f"dist btwn: {np.linalg.norm(g_pos - o_pos)}")
                return False
                
        print(f"dist btwn: {np.linalg.norm(g_pos - o_pos)}")
        return True
    
    def get_pose_relative_to_object(self, obj):
        """Get gripper pose (x, y, z, roll, pitch, yaw) relative to object."""
        gripper_pos, gripper_orn = p.getBasePositionAndOrientation(self.id)
        obj_pos, obj_orn = p.getBasePositionAndOrientation(obj.id)
        
        # Relative position
        rel_pos = np.array(gripper_pos) - np.array(obj_pos)
        
        # Relative orientation (simplified - just use gripper orientation)
        gripper_euler = p.getEulerFromQuaternion(gripper_orn)
        
        return (*rel_pos, *gripper_euler)
    
    def remove(self):
        """Remove gripper from simulation."""
        if self.constraint_id is not None:
            p.removeConstraint(self.constraint_id)
        if self.id is not None:
            p.removeBody(self.id)
    
    @staticmethod
    def get_random_start_position(radius=0.5, min_height=0.15):
        """Generate random position on hemisphere around origin."""
        theta = np.random.uniform(0, 2 * np.pi)
        phi = np.random.uniform(0, np.pi / 3)  # limit to upper hemisphere
        
        x = radius * np.sin(phi) * np.cos(theta)
        y = radius * np.sin(phi) * np.sin(theta)
        z = radius * np.cos(phi)
        
        z = max(z, min_height)  # ensure minimum height
        
        return np.array([x, y, z])
    
    @staticmethod
    def orient_towards_origin(pos, add_noise=False):
        """Calculate orientation (roll, pitch, yaw) to face the origin."""
        x, y, z = pos
        
        yaw = np.arctan2(-y, -x)
        distance_xy = np.sqrt(x**2 + y**2)
        pitch = np.arctan2(z, distance_xy)
        
        if add_noise:
            roll = np.random.uniform(-0.15, 0.15)
            pitch += np.random.uniform(-0.1, 0.1)
            yaw += np.random.uniform(-0.1, 0.1)
        else:
            roll = 0.0
        
        return np.array([roll, pitch, yaw])


class TwoFingerGripper(Gripper):
    """PR2 two-finger parallel jaw gripper."""
    
    def __init__(self, position=(0, 0, 0), orientation=(0, 0, 0)):
        super().__init__("pr2_gripper.urdf", position, orientation)
        self.name = "TwoFingerGripper"
        self.finger_joints = [0, 2]
        
    def _initialize_joints(self):
        """Set initial joint positions for PR2 gripper."""
        initial_positions = [0.55, 0.0, 0.55, 0.0]
        for i, pos in enumerate(initial_positions):
            p.resetJointState(self.id, i, pos)
    
    def get_finger_joints(self):
        return self.finger_joints
    
    def open(self, target=0.55):
        """Open gripper fingers."""
        for joint in self.finger_joints:
            p.setJointMotorControl2(
                self.id, joint, p.POSITION_CONTROL,
                targetPosition=target, maxVelocity=1, force=30
            )
    
    def close(self, target=0.05):
        """Close gripper fingers."""
        for joint in self.finger_joints:
            p.setJointMotorControl2(
                self.id, joint, p.POSITION_CONTROL,
                targetPosition=target, maxVelocity=2, force=400
            )
    
    def _apply_grip_force(self):
        """Apply continuous grip force."""
        for joint in self.finger_joints:
            p.setJointMotorControl2(
                self.id, joint, p.POSITION_CONTROL,
                targetPosition=0.05, maxVelocity=2, force=400
            )


class ThreeFingerGripper(Gripper):
    """SDH three-finger gripper."""
    
    def __init__(self, position=(0, 0, 0), orientation=(0, 0, 0)):
        urdf_path = os.path.join(os.path.dirname(__file__), "objects", "sdh", "sdh.urdf")
        super().__init__(urdf_path, position, orientation)
        self.name = "ThreeFingerGripper"
        self.finger_joints = []
        self.base_orientation = np.array([math.pi, 0, 0])  # default orientation is now upside down
        
    def _initialize_joints(self):
        """Set initial joint positions for SDH gripper."""
        self.num_joints = p.getNumJoints(self.id)
        self.finger_joints = list(range(self.num_joints))
        
        for i in self.finger_joints:
            p.resetJointState(self.id, i, -0.5)
    
    @staticmethod  
    def orient_towards_origin(pos, add_noise=False):
        """Calculate orientation to face origin, with base upside-down rotation."""
        x, y, z = pos
        
        yaw = np.arctan2(-y, -x)
        distance_xy = np.sqrt(x**2 + y**2)
        pitch = np.arctan2(z, distance_xy)
        
        if add_noise:
            roll = np.random.uniform(-0.15, 0.15)
            pitch += np.random.uniform(-0.1, 0.1)
            yaw += np.random.uniform(-0.1, 0.1)
        else:
            roll = 0.0
        
        
        roll += math.pi  # base upside down orientation
        
        return np.array([roll, pitch, yaw])
    
    def get_finger_joints(self):
        return self.finger_joints
    
    def open(self, target=-0.5):
        """Open all fingers."""
        for joint in self.finger_joints:
            p.setJointMotorControl2(
                self.id, joint, p.POSITION_CONTROL,
                targetPosition=target, maxVelocity=1, force=30
            )
    
    def close(self, target=0.3):
        """Close all fingers."""
        for joint in self.finger_joints:
            p.setJointMotorControl2(
                self.id, joint, p.POSITION_CONTROL,
                targetPosition=target, maxVelocity=2, force=400
            )
    
    def _apply_grip_force(self):
        """Apply continuous grip force."""
        for joint in self.finger_joints:
            p.setJointMotorControl2(
                self.id, joint, p.POSITION_CONTROL,
                targetPosition=0.3, maxVelocity=2, force=400
            )

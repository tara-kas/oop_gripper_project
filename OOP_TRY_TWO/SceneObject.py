import pybullet as p
from abc import ABC, abstractmethod


class SceneObject(ABC):
    """Base class for any object in the PyBullet scene."""
    
    def __init__(self, urdf, position=(0, 0, 0), orientation=(0, 0, 0), scale=1.0):
        self.urdf = urdf
        self._position = position
        self._orientation_euler = orientation
        self._orientation_quat = p.getQuaternionFromEuler(orientation)
        self.scale = scale
        self.id = None
        self.name = "SceneObject"
        
    @property
    def position(self):
        return self._position
    
    @position.setter
    def position(self, value):
        self._position = value
        
    @property
    def orientation(self):
        return self._orientation_quat
    
    @orientation.setter
    def orientation(self, value):
        if len(value) == 3:
            self._orientation_euler = value
            self._orientation_quat = p.getQuaternionFromEuler(value)
        else:
            self._orientation_quat = value
            self._orientation_euler = p.getEulerFromQuaternion(value)
    
    def load(self):
        """Load object into the PyBullet scene."""
        self.id = p.loadURDF(
            self.urdf, 
            self._position, 
            self._orientation_quat, 
            globalScaling=self.scale
        )
        return self.id
    
    def reset(self):
        """Reset object to its initial position."""
        p.resetBasePositionAndOrientation(self.id, self._position, self._orientation_quat)
    
    def get_current_pose(self):
        """Get current position and orientation from simulation."""
        pos, orn = p.getBasePositionAndOrientation(self.id)
        return pos, orn
    
    @abstractmethod
    def get_grasp_offset(self):
        """Return the recommended approach offset for grasping."""
        pass


class Cube(SceneObject):
    """Cube object for grasping experiments."""
    
    def __init__(self, urdf, position=(0, 0, 0), orientation=(0, 0, 0), scale=0.8):
        super().__init__(urdf, position, orientation, scale)
        self.name = "Cube"
        self.id = self.load()
        self._compute_grasp_params()
        
    def _compute_grasp_params(self):
        """Compute grasp-related parameters from object geometry."""
        aabb_min, aabb_max = p.getAABB(self.id)
        self.grasp_height = aabb_max[2]
        self.half_extent = (aabb_max[0] - aabb_min[0]) / 2
        
    def get_grasp_offset(self):
        """Half diagonal plus margin for approach."""
        return self.half_extent * 1.5 + 0.08


class Cylinder(SceneObject):
    """Cylinder object for grasping experiments."""
    
    def __init__(self, urdf, position=(0, 0, 0), orientation=(0, 0, 0), scale=0.8):
        super().__init__(urdf, position, orientation, scale)
        self.name = "Cylinder"
        self.id = self.load()
        self._compute_grasp_params()
        
    def _compute_grasp_params(self):
        """Compute grasp-related parameters from object geometry."""
        aabb_min, aabb_max = p.getAABB(self.id)
        self.grasp_height = aabb_max[2]
        self.radius = (aabb_max[0] - aabb_min[0]) / 2
        
    def get_grasp_offset(self):
        """Radius plus margin for approach."""
        return self.radius + 0.1

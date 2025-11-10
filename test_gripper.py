import pybullet as p
import pybullet_data
import time


# ---------- Base Classes ----------
class SceneObject:
    """Base class for any object in the PyBullet scene."""
    def __init__(self, urdf_file, position, orientation=(0, 0, 0)):
        self.urdf_file = urdf_file
        self.position = position
        self.orientation = p.getQuaternionFromEuler(orientation)
        self.id = None
        self.name = None

    def load(self):
        """Load the object into the scene."""
        self.id = p.loadURDF(self.urdf_file, self.position, self.orientation)
        return self.id


class Box(SceneObject):
    """Box object inheriting from SceneObject."""
    def __init__(self, position):  # MISSING: Add position parameter
        super().__init__("cube_small.urdf", position)  # MISSING: Call parent constructor
        self.grasp_height = 0.03
        self.name = None  # Will be set later

    def update_name(self, obj_id):
        self.name = f"Box_{obj_id}"

class Cylinder(SceneObject):
    """Cylinder object inheriting from SceneObject."""
    def __init__(self, position):  # MISSING: Add position parameter
        super().__init__("cylinder.urdf", position)  # MISSING: Call parent constructor
        self.grasp_height = 0.1
        self.name = None  # Will be set later

    def update_name(self, obj_id):
        self.name = f"Cylinder_{obj_id}"

 

# ---------- Gripper Classes ----------
class Gripper:
    """Base gripper class that defines common gripper behavior."""
    def __init__(self, urdf_path, base_position):
        self.urdf_path = urdf_path
        self.base_position = base_position
        self.id = None
        self.constraint_id = None
        self.grasp_moving = False

    def load(self):
        """Load gripper into the PyBullet world."""
        self.id = p.loadURDF(self.urdf_path, *self.base_position)
        return self.id

    def attach_fixed(self, offset):
        """Attach gripper to a fixed world position."""
        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.id,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=offset,
            childFramePosition=self.base_position
        )

    def move(self, z, yaw=0.0):
        """Move gripper to a new height and yaw."""
        
        if self.constraint_id is None:
            raise ValueError("Gripper must be fixed before moving.")
        p.changeConstraint(
            self.constraint_id,
            jointChildPivot=[0.5, 0.3, z],
            jointChildFrameOrientation=p.getQuaternionFromEuler([0, 0, yaw]),
            maxForce=50
        )


 
    def update_camera(self, z, yaw):
        p.resetDebugVisualizerCamera(
            cameraDistance=0.5,
            cameraYaw=50 + (yaw * 180 / 3.1416),
            cameraPitch=-60,
            cameraTargetPosition=[0.5, 0.3, z]
        )

class TwoFingerGripper(Gripper):
    """A specific two-finger gripper inheriting from Gripper class.
    Initialize the gripper using default base_position=(0.5, 0.3, 0.7) and the urdf name: pr2_gripper.urdf setting up the relevant attributes.
    You need to use the parent's initializer accordingly.
    """ 
    def __init__(self, position=(0.5, 0.3, 0.7)):  # FIXED: Remove orientation parameter
        super().__init__("pr2_gripper.urdf", position)  # FIXED: Remove orientation
        self.name = "TwoFingerGripper"
        self.finger_joints = [0, 2]  # FIXED: Use [0, 2] consistently
        self.max_force = 100
        self.grasp_force = 50

    def start(self):
        """Open gripper at start."""
        initial_positions = [0.550569, 0.0, 0.549657, 0.0]
        for i, pos in enumerate(initial_positions):
            p.resetJointState(self.id, i, pos)

    def open(self):
        """Open the gripper fingers."""
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=0.0, maxVelocity=1, force=10)

    def close(self):
        """Close the gripper fingers."""
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=0.1, maxVelocity=1, force=10)

    def grasp_and_lift(self, obj, lift_height=0.4, lift_steps=150):
        """Perform grasping and lifting sequence with sustained gripping force."""
        yaw_angle = 0.0

        grasp_height = obj.grasp_height

        # --- Set friction on contact surfaces ---
        p.changeDynamics(obj.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)
        p.changeDynamics(self.id, -1, lateralFriction=2.0, rollingFriction=0.1, spinningFriction=0.1)

        # --- Move above object by calling the move function inherited from the parent
        self.move(obj.position[2] + 0.1, yaw_angle)
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)

        # --- Lower onto object ---
        self.move(grasp_height, yaw_angle)
        print("\033[93mmove to the grasp height")
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)

        # --- Close gripper strongly ---
        for joint in [0, 2]:
            p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                    targetPosition=0.12, force=300, maxVelocity=2)
        for _ in range(100):  # allow contact to form
            p.stepSimulation()
            time.sleep(1./240.)

        # --- Continuous hold while lifting step-by-step ---
        z_current = grasp_height
        z_target = lift_height
        z_step = (z_target - z_current) / lift_steps

        for _ in range(lift_steps):
            z_current += z_step
            self.move(z_current, yaw_angle)

            # Continuously reapply strong grip to prevent slip
            for joint in [0, 2]:
                p.setJointMotorControl2(self.id, joint, p.POSITION_CONTROL,
                                        targetPosition=0.12, force=400, maxVelocity=2)

            p.stepSimulation()
            time.sleep(1./240.)



# ---------- Environment Setup ----------
def setup_environment():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -10)
    p.setRealTimeSimulation(0)
    plane_id = p.loadURDF("plane.urdf")
    return plane_id


# ---------- Main Simulation ----------
if __name__ == "__main__":
    setup_environment()

    objects = [Box([0.6, 0.3, 0.025]), Cylinder([0.6, 0.3, 0.0])]
 
    for i in range(len(objects)):
        obj_id = objects[i].load()
        objects[i].update_name(obj_id)
        # Create and set up the gripper
        gripper = TwoFingerGripper()
        gripper.load()
        gripper.start()
        gripper.attach_fixed(offset=[0.2, 0, 0])
        gripper.update_camera(z=0.7, yaw=0.0)

        # Grasp and lift the box
        gripper.grasp_and_lift(objects[i])

        # Keep GUI open briefly
        for _ in range(100):
            p.stepSimulation()
            time.sleep(1./240.)
        print(f"\033[91mdone grasping: {objects[i].name}")

        # --- Remove the gripper before next iteration ---
        if gripper.constraint_id is not None:
            p.removeConstraint(gripper.constraint_id)
        p.removeBody(gripper.id)

        p.removeBody(objects[i].id)

        # short pause before loading next one
        time.sleep(0.5)

    p.disconnect()
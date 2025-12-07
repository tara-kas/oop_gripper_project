"""
Grasp Data Collection Pipeline
Generates grasp dataset by sampling random gripper poses and testing grasp success.
"""

import math
import pybullet as p
import pybullet_data
import time
import os
import numpy as np
import pandas as pd
from SceneObject import Cube, Cylinder
from Gripper import TwoFingerGripper, ThreeFingerGripper


def setup_environment(gui=True):
    """Initialize PyBullet environment."""
    if gui:
        physics_client = p.connect(p.GUI)
    else:
        physics_client = p.connect(p.DIRECT)
    
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    
    # Load ground plane
    plane_id = p.loadURDF("plane.urdf")
    
    # Set camera for better view
    p.resetDebugVisualizerCamera(
        cameraDistance=1.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.1]
    )
    
    return physics_client


def collect_grasp_data(gripper_class, obj, num_samples, radius=1, add_noise=True):
    """
    Collect grasp data for a specific gripper-object combination.
    
    Args:
        gripper_class: Class of gripper to use (TwoFingerGripper or ThreeFingerGripper)
        obj: SceneObject instance to grasp
        num_samples: Number of grasp attempts
        radius: Distance from object for initial gripper position
        add_noise: Whether to add noise to orientations
    
    Returns:
        List of dictionaries with grasp data
    """
    data = []
    
    for i in range(num_samples):
        print(f"\n{'='*50}")
        print(f"Trial {i+1}/{num_samples} - {gripper_class.__name__} grasping {obj.name}")
        print(f"{'='*50}")
        
        # Reset object to origin
        obj.reset()
        for _ in range(20):
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Generate random start position
        start_pos = gripper_class.get_random_start_position(radius=radius)
        
        # Calculate orientation to face the object
        orientation = gripper_class.orient_towards_origin(start_pos, add_noise=add_noise)
        
        print(f"Start position: {start_pos}")
        print(f"Orientation (rpy): {orientation}")
        
        # Create and load gripper
        gripper = gripper_class(position=start_pos, orientation=orientation)
        gripper.load()

        print(f"Gripper orientation - Roll: {orientation[0]:.3f}, Pitch: {orientation[1]:.3f}, Yaw: {orientation[2]:.3f}")
        gripper_pos, gripper_orn = p.getBasePositionAndOrientation(gripper.id)
        rot_matrix = p.getMatrixFromQuaternion(gripper_orn)
        z_axis = np.array([rot_matrix[2], rot_matrix[5], rot_matrix[8]]) 
        z_end = np.array(gripper_pos) + z_axis * 0.2
        p.addUserDebugLine(gripper_pos, z_end, [0, 0, 1], 3, lifeTime=10)  
        p.addUserDebugLine(gripper_pos, obj.position, [1, 0, 0], 2, lifeTime=10) 

        gripper.create_constraint()
        
        # Allow gripper to settle
        for _ in range(10):
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Record pose BEFORE grasping (relative to object)
        pose = gripper.get_pose_relative_to_object(obj)
        
        # Execute grasp and lift
        approach_offset = obj.get_grasp_offset()
        success = gripper.grasp_and_lift(
            obj, 
            approach_offset=approach_offset,
            lift_height=0.35,
            hold_time=3.0
        )
        print(f"success: {success}")
        
        print(f"Grasp {'SUCCESS' if success else 'FAILURE'}")
        
        # Store data
        data.append({
            'gripper_type': gripper_class.__name__,
            'object_type': obj.name,
            'rel_x': pose[0],
            'rel_y': pose[1],
            'rel_z': pose[2],
            'roll': pose[3],
            'pitch': pose[4],
            'yaw': pose[5],
            'success': int(success)
        })
        
        # Clean up gripper
        gripper.remove()
        
        # Brief pause between trials
        time.sleep(0.1)
    
    return data


def balance_dataset(df, target_per_class=None):
    """
    Balance dataset to have equal positive and negative samples.
    
    Args:
        df: pandas DataFrame with 'success' column
        target_per_class: Target samples per class (None = use minority class size)
    
    Returns:
        Balanced DataFrame
    """
    success_df = df[df['success'] == 1]
    failure_df = df[df['success'] == 0]
    
    if target_per_class is None:
        target_per_class = min(len(success_df), len(failure_df))
    
    if len(success_df) > target_per_class:
        success_df = success_df.sample(n=target_per_class, random_state=42)
    if len(failure_df) > target_per_class:
        failure_df = failure_df.sample(n=target_per_class, random_state=42)
    
    balanced_df = pd.concat([success_df, failure_df]).sample(frac=1, random_state=42)
    return balanced_df.reset_index(drop=True)


def main():
    # Configuration
    SAMPLES_PER_COMBINATION = 30  # 30 samples x 4 combinations = 120 total
    RADIUS = 2  # Distance from object for starting position
    USE_GUI = True
    
    # Paths to object URDFs
    base_path = os.path.dirname(__file__)
    cube_urdf = os.path.join(base_path, "objects", "cube_small.urdf")
    cylinder_urdf = os.path.join(base_path, "objects", "cylinder.urdf")
    
    # If local URDFs don't exist, use pybullet_data
    if not os.path.exists(cube_urdf):
        cube_urdf = "cube_small.urdf"
    if not os.path.exists(cylinder_urdf):
        cylinder_urdf = "cylinder.urdf"
    
    # Setup environment
    print("Setting up PyBullet environment...")
    setup_environment(gui=USE_GUI)
    
    all_data = []
    
    # Define gripper-object combinations
    gripper_classes = [TwoFingerGripper, ThreeFingerGripper]
    object_configs = [
        (Cube, cube_urdf, (0, 0, 0.025)),      # Cube at origin
        (Cylinder, cylinder_urdf, (0, 0, 0.02))  # Cylinder at origin
    ]
    
    # Collect data for each combination
    for gripper_class in gripper_classes:
        for obj_class, urdf, position in object_configs:
            print(f"\n{'#'*60}")
            print(f"Collecting data: {gripper_class.__name__} + {obj_class.__name__}")
            print(f"{'#'*60}")
            
            # Create object
            obj = obj_class(urdf, position=position)
            
            # Collect grasp data
            data = collect_grasp_data(
                gripper_class, 
                obj, 
                num_samples=SAMPLES_PER_COMBINATION,
                radius=RADIUS,
                add_noise=True
            )
            all_data.extend(data)
            
            # Remove object
            p.removeBody(obj.id)
    
    # Create DataFrame
    df = pd.DataFrame(all_data)
    
    print("\n" + "="*60)
    print("DATA COLLECTION COMPLETE")
    print("="*60)
    
    # Print statistics
    print(f"\nTotal samples collected: {len(df)}")
    print(f"Successful grasps: {df['success'].sum()}")
    print(f"Failed grasps: {len(df) - df['success'].sum()}")
    print(f"Success rate: {df['success'].mean()*100:.1f}%")
    
    # Balance the dataset
    balanced_df = balance_dataset(df)
    
    print(f"\nBalanced dataset: {len(balanced_df)} samples")
    print(f"  Successful: {balanced_df['success'].sum()}")
    print(f"  Failed: {len(balanced_df) - balanced_df['success'].sum()}")
    
    # Save to CSV
    output_path = os.path.join(base_path, "grasp_dataset.csv")
    balanced_df.to_csv(output_path, index=False)
    print(f"\nDataset saved to: {output_path}")
    
    # Display sample of data
    print("\nSample of collected data:")
    print(balanced_df.head(10).to_string())
    
    # Keep simulation running briefly to view final state
    print("\nSimulation complete. Closing in 5 seconds...")
    for _ in range(int(5 * 240)):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
    return balanced_df


if __name__ == "__main__":
    df = main()

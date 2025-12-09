""" grasp data collection functions"""

import pybullet as p
import numpy as np
import pandas as pd
import time

LIFT_HEIGHT = 0.35
HOLD_TIME = 3.0

def _collect_grasp_data(gripper_class, obj, num_samples, radius=1, add_noise=True):
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

def collect_all_training_data(gripper_classes, object_configs, samples_per_combination, radius):
    """
    Collect training data for all gripper-object combinations.
    
    Args:
        gripper_classes: List of gripper classes to use
        object_configs: List of tuples (ObjectClass, urdf_path, position)
        samples_per_combination: Number of samples per gripper-object pair
        radius: Starting distance from object
        
    Returns:
        DataFrame with all collected data
    """
    all_data = {}
    
    for gripper_class in gripper_classes:
        for obj_class, urdf, position in object_configs:
            print(f"\n{'#'*60}")
            print(f"Collecting data: {gripper_class.__name__} + {obj_class.__name__}")
            print(f"{'#'*60}")
            
            obj = obj_class(urdf, position=position)
            
            data = _collect_grasp_data(
                gripper_class,
                obj,
                num_samples=samples_per_combination,
                radius=radius,
                add_noise=True
            )
            key = f"{gripper_class.__name__}_{obj_class.__name__}"
            all_data[key] = pd.DataFrame(data)
            
            p.removeBody(obj.id)
    
    return all_data


def print_dataset_statistics(df, title="Dataset Statistics"):
    """Print statistics about the dataset."""
    print(f"\n{title}")
    print(f"{'-'*40}")
    print(f"Total samples: {len(df)}")
    print(f"Successful grasps: {df['success'].sum()}")
    print(f"Failed grasps: {len(df) - df['success'].sum()}")
    print(f"Success rate: {df['success'].mean()*100:.1f}%")
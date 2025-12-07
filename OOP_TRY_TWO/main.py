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

from config import (
    BASE_PATH, CUBE_URDF, CYLINDER_URDF, 
    SAMPLES_PER_COMBINATION, RADIUS, USE_GUI,
    DATASET_PATH, setup_environment
)

from data_collection import (
    collect_all_training_data,
    print_dataset_statistics
)

def main():
    global CUBE_URDF, CYLINDER_URDF

    
    # Paths to object URDFs
    base_path = os.path.dirname(__file__)
    cube_urdf = os.path.join(base_path, "objects", "cube_small.urdf")
    cylinder_urdf = os.path.join(base_path, "objects", "duck_vhacd.urdf")
    
    # If local URDFs don't exist, use pybullet_data
    if not os.path.exists(cube_urdf):
        cube_urdf = "cube_small.urdf"
    if not os.path.exists(cylinder_urdf):
        cylinder_urdf = "duck_vhacd.urdf"
    
    # setup environment
    print("Setting up PyBullet environment...")
    setup_environment(gui=USE_GUI)
    
    all_data = []
    
    # Define gripper-object combinations
    gripper_classes = [TwoFingerGripper, ThreeFingerGripper]
    object_configs = [
        (Cube, cube_urdf, (0, 0, 0.025)),      # Cube at origin
        # (Cylinder, cylinder_urdf, (0, 0, 0.02))  # Cylinder at origin
    ]
    
    # collect data
    print("\n" + "="*60)
    print("DATA COLLECTION")
    print("="*60)
    
    df = collect_all_training_data(
        gripper_classes,
        object_configs,
        SAMPLES_PER_COMBINATION,
        RADIUS
    )
    
    # Print statistics
    print_dataset_statistics(df, "Data Collection Complete")
    
    # Save to CSV
    df.to_csv(DATASET_PATH, index=False)
    print(f"\nDataset saved to: {DATASET_PATH}")
    
    # cleanup
    print("\nSimulation complete. Closing in 3 seconds...")
    for _ in range(int(3 * 240)):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
    return df

if __name__ == "__main__":
    df = main()
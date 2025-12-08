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
    BASE_PATH, CUBE_URDF, DUCK_URDF, 
    SAMPLES_PER_COMBINATION, RADIUS, USE_GUI,
    DATASET_PATH, MODEL_PATH, setup_environment
)

from data_collection import (
    collect_all_training_data,
    print_dataset_statistics
)

from classify import (
    load_dataset,
    train_classifier,
    predict_grasp
)

def main():
    global CUBE_URDF, DUCK_URDF
    
    # If local URDFs don't exist, use pybullet_data
    if not os.path.exists(CUBE_URDF):
        CUBE_URDF = "cube_small.urdf"
    if not os.path.exists(DUCK_URDF):
        DUCK_URDF = "duck_vhacd.urdf"
    
    # setup environment
    print("Setting up PyBullet environment...")
    setup_environment(gui=USE_GUI)
    
    all_data = []
    
    # Define gripper-object combinations
    gripper_classes = [TwoFingerGripper, ThreeFingerGripper]
    object_configs = [
        (Cube, CUBE_URDF, (0, 0, 0.025)),      # Cube at origin
        # (Duck, duck_urdf, (0, 0, 0.02))  # Cylinder at origin
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
    
    # print statistics
    print_dataset_statistics(df, "Data Collection Complete")
    
    # save to csv
    df.to_csv(DATASET_PATH, index=False)
    print(f"\nDataset saved to: {DATASET_PATH}")
    
    # train classifier
    print("\n" + "="*60)
    print("TRAINING CLASSIFIER")
    print("="*60)
    
    csv_paths = [
        os.path.join(BASE_PATH, "grasp_dataset_cube.csv"),
        os.path.join(BASE_PATH, "grasp_dataset_duck.csv")
    ]
    
    csv_paths = [p for p in csv_paths if os.path.exists(p)]
    
    # load in csv to pandas df
    balanced_df = load_dataset(csv_paths)
    print(f"Balanced dataset: {len(balanced_df)} samples ({balanced_df['success'].sum()} positive)")
    
    clf, features = train_classifier(balanced_df, MODEL_PATH)
    
    # cleanup
    print("\nSimulation complete. Closing in 3 seconds...")
    for _ in range(int(3 * 240)):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
    return df

if __name__ == "__main__":
    df = main()
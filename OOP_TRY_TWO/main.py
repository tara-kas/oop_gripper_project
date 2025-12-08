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

from SceneObject import Cube, Duck
from Gripper import TwoFingerGripper, ThreeFingerGripper

from config import (
    BASE_PATH, CUBE_URDF, DUCK_URDF, 
    SAMPLES_PER_COMBINATION, RADIUS, USE_GUI,
    MODEL_PATHS, setup_environment
)

from data_collection import (
    collect_all_training_data,
    print_dataset_statistics
)

from classify import (
    load_dataset,
    train_classifier,
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
    gripper_classes = [TwoFingerGripper]
    object_configs = [
        (Cube, CUBE_URDF, (0, 0, 0.025)),      # Cube at origin
        # (Duck, DUCK_URDF, (0, 0, 0.02))        # Duck at origin
    ]
    
    # collect data
    print("\n" + "="*60)
    print("DATA COLLECTION")
    print("="*60)
    
    dfs = collect_all_training_data(
        gripper_classes,
        object_configs,
        SAMPLES_PER_COMBINATION,
        RADIUS
    )
    
    # save to csv and print statistics for each df
    csv_paths = []
    for key, df in dfs.items():
        print_dataset_statistics(df, f"{key} Statistics")
        path = os.path.join(BASE_PATH, f"grasp_dataset_{key}.csv")
        df.to_csv(path, index=False)
        csv_paths.append(path)
        print(f"Saved: {path}")
    
    # train classifier
    print("\n" + "="*60)
    print("TRAINING CLASSIFIER")
    print("="*60)

    # load in csv to pandas df
    for i, csv_path in enumerate(csv_paths):
        balanced_df = load_dataset(csv_path)
        print(f"Balanced dataset for {csv_path}: {len(balanced_df)} samples ({balanced_df['success'].sum()} positive)")
        clf, features = train_classifier(balanced_df, MODEL_PATHS[i])
    
    # cleanup
    print("\nSimulation complete. Closing in 3 seconds...")
    for _ in range(int(3 * 240)):
        p.stepSimulation()
        time.sleep(1./240.)
    
    p.disconnect()
    
    # return dfs

if __name__ == "__main__":
    df = main()
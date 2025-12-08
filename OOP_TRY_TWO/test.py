"""testing phase: evaluate classifier predictions against actual grasp execution."""

import os
import pybullet as p
import numpy as np
import joblib
import time

from config import BASE_PATH, CUBE_URDF, DUCK_URDF, RADIUS, setup_environment, MODEL_PATHS
from SceneObject import Cube, Duck
from Gripper import TwoFingerGripper, ThreeFingerGripper


def predict_grasp(clf, pose, features):
    """Predict grasp success for a single pose dict."""
    X = np.array([[pose[f] for f in features]])
    return clf.predict(X)[0], clf.predict_proba(X)[0]

def test_classifier(clf, features, gripper_class, obj_class, obj_urdf, obj_position, num_tests=10):
    """
    Test classifier by predicting grasp success then executing to verify.
    
    Arguments:
        clf: Trained classifier
        features: List of feature names
        gripper_class: TwoFingerGripper or ThreeFingerGripper
        obj_class: Cube or Duck
        obj_urdf: Path to object URDF
        obj_position: Starting position for object
        num_tests: Number of test grasps
    
    Returns:
        Dict with test results
    """
    
    # initialise 
    correct = 0
    results = []
    
    for i in range(num_tests):
        print(f"\n{'='*50}")
        print(f"Test {i+1}/{num_tests}: {gripper_class.__name__} + {obj_class.__name__}")
        print(f"{'='*50}")
        
        # Create object
        obj = obj_class(obj_urdf, position=obj_position)
        
        # Reset object
        p.resetBasePositionAndOrientation(obj.id, obj_position, p.getQuaternionFromEuler((0,0,0)))
        for _ in range(20):
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Generate random grasp pose
        start_pos = gripper_class.get_random_start_position(radius=RADIUS)
        orientation = gripper_class.orient_towards_origin(start_pos, add_noise=True)
        
        # Create gripper
        gripper = gripper_class(position=start_pos, orientation=orientation)
        gripper.load()
        gripper.create_constraint()
        
        for _ in range(10):
            p.stepSimulation()
            time.sleep(1./240.)
        
        # Get pose relative to object
        pose = gripper.get_pose_relative_to_object(obj)
        pose_dict = {
            'rel_x': pose[0], 'rel_y': pose[1], 'rel_z': pose[2],
            'roll': pose[3], 'pitch': pose[4], 'yaw': pose[5]
        }
        
        # Predict using classifier
        prediction, proba = predict_grasp(clf, pose_dict, features)
        print(f"Predicted: {'SUCCESS' if prediction else 'FAILURE'} (confidence: {max(proba)*100:.1f}%)")
        
        # Execute actual grasp
        actual = gripper.grasp_and_lift(obj, approach_offset=obj.get_grasp_offset(), lift_height=0.35, hold_time=3.0)
        print(f"Actual: {'SUCCESS' if actual else 'FAILURE'}")
        
        # Check if prediction was correct
        is_correct = (prediction == int(actual))
        if is_correct:
            correct += 1
        print(f"Prediction: {'CORRECT' if is_correct else 'WRONG'}")
        
        results.append({
            'predicted': prediction,
            'actual': int(actual),
            'correct': is_correct,
            'confidence': max(proba)
        })
        
        # cleanup
        gripper.remove()
        p.removeBody(obj.id)
        time.sleep(0.1)
    
    accuracy = correct / num_tests
    print(f"\n{'='*50}")
    print(f"RESULTS: {correct}/{num_tests} correct ({accuracy*100:.1f}%)")
    print(f"{'='*50}")
    
    return {'accuracy': accuracy, 'correct': correct, 'total': num_tests, 'results': results}

if __name__=="__main__":
    
    setup_environment(gui=False)
    features = ['rel_x', 'rel_y', 'rel_z', 'roll', 'pitch', 'yaw']
    clf = joblib.load(MODEL_PATHS[0])
    
    print("\n" + "="*60)
    print("TESTING CLASSIFIER")
    print("="*60)
    
    combinations = [
        (TwoFingerGripper, Cube, CUBE_URDF, (0, 0, 0.025)),
        (ThreeFingerGripper, Cube, CUBE_URDF, (0, 0, 0.025)),
        (TwoFingerGripper, Duck, DUCK_URDF, (0, 0, 0.02)),
        (ThreeFingerGripper, Duck, DUCK_URDF, (0, 0, 0.02)),
    ]
    
    for gripper_class, obj_class, urdf, position in combinations:
        print(f"\n--- {gripper_class.__name__} + {obj_class.__name__} ---")
        test_classifier(clf, features, gripper_class, obj_class, urdf, position, num_tests=10)
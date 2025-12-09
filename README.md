# Grasp Data Collection and Classification Project

This project provides a pipeline for collecting grasp data, training classifiers, and evaluating robotic grippers' grasping capabilities using PyBullet. The project is modular, with components for data collection, classification, and testing.

## Project Structure

- **`main.py`**: The entry point of the project. It sets up the PyBullet environment, defines gripper-object combinations, collects training data, trains classifiers, and evaluates results.
- **`classify.py`**: Contains functions for training and evaluating a Random Forest classifier. It balances datasets, tunes hyperparameters, and saves trained models.
- **`data_collection.py`**: Implements functions for collecting grasp data for various gripper-object combinations. Includes utilities for generating random poses and recording grasp success.
- **`Gripper.py`**: Defines the `Gripper` abstract base class and its derived classes for robotic grippers. Includes methods for loading URDFs, initializing joints, and controlling grippers.
- **`SceneObject.py`**: Defines the `SceneObject` base class for objects in the PyBullet scene. Includes methods for loading URDFs, resetting positions, and managing object properties.
- **`test.py`**: Contains functions for testing trained classifiers. It predicts grasp success for given poses and verifies predictions through actual grasp executions in PyBullet.
- **`config.py`**: Stores configuration constants and helper functions for setting up the PyBullet environment and managing file paths.

## Pipeline

1. **Grasp Data Collection**:
   - Collects grasp data for different gripper-object combinations.
   - Records pose features and grasp success labels.

2. **Classification**:
   - Trains a Random Forest classifier on collected data.
   - Balances datasets to ensure equal success/failure samples.
   - Saves trained models for future use.

3. **Testing**:
   - Evaluates classifier predictions against actual grasp executions.
   - Generates confusion matrices to visualize performance.

## Dependencies

The project requires the following Python libraries:
- `pybullet`
- `numpy`
- `pandas`
- `scikit-learn`
- `joblib`
- `pybullet_data`

Install dependencies using:
```bash
pip install -r requirements.txt
```

## Usage

1. **Run the main pipeline**:
   ```bash
   python3 main.py
   ```
   This will set up the environment, collect data, train classifiers, and evaluate results.

2. **Train a classifier**:
   Use the `train_classifier` function in `classify.py` to train a Random Forest model on collected data.

3. **Test a classifier**:
   Use the `test_classifier` function in `test.py` to evaluate a trained classifier's predictions against actual grasp executions.

## File Structure

- `data/`: Contains collected grasp datasets.
- `objects/`: Stores URDF files for objects and grippers.
- `clf/`: Stores trained classifier models.
- `confusion_matrix/`: Stores confusion matrices generated during testing.

## Acknowledgments

For COMP0213. Thank you to Yasemin and Daniel.

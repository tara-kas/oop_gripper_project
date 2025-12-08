# grasp classifier model using Random Forest
import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import joblib

from config import MODEL_PATHS

def load_dataset(csv_path):
    # csv_path is type list so iterate over and concat
    df = pd.read_csv(csv_path)
    
    # balance dataset so equal success/failure samples
    success_df = df[df['success'] == 1]
    fail_df = df[df['success'] == 0]
    min_samples = min(len(success_df), len(fail_df))
    balanced_df = pd.concat([success_df.sample(min_samples, random_state=42), fail_df.sample(min_samples, random_state=42)])
    
    return balanced_df

def train_classifier(df, model_path=None):
    """ train random forest on pose features """
    features = ['rel_x', 'rel_y', 'rel_z', 'roll', 'pitch', 'yaw']
    
    X = df[features].values
    Y = df['success'].values
    
    X_train, X_test, Y_train, Y_test = train_test_split(X, Y, test_size=0.2, random_state=42)
    
    # hyperparam tuning
    clf = RandomForestClassifier(
        n_estimators=100,      # no. of trees
        max_depth=10,        # tree depth
        min_samples_split=2,   # min samples to split node
        min_samples_leaf=1,    # min samples per leaf
        random_state=42)
    
    clf.fit(X_train, Y_train)
    
    train_acc = accuracy_score(Y_train, clf.predict(X_train))
    test_acc = accuracy_score(Y_test, clf.predict(X_test))
    print(f"Train accuracy: {train_acc*100:.1f}%, Validation accuracy: {test_acc*100:.1f}%")
    
    if model_path:
        joblib.dump(clf, model_path)
        print(f"model saved to {model_path}")
        
    return clf, features

if __name__ == "__main__":
    # tune each combination w/ own hyperparams
    csv_paths = ["grasp_dataset_TwoFingerGripper_Cube.csv"]
    # csv_paths = ["grasp_dataset_TwoFingerGripper_Cube.csv", "grasp_dataset_TwoFingerGripper_Duck.csv", "grasp_dataset_ThreeFingerGripper_Cube.csv", "grasp_dataset_ThreeFingerGripper_Duck.csv"]
    for i, csv_path in enumerate(csv_paths):
        balanced_df = load_dataset(csv_path)
        print(f"Balanced dataset for {csv_path}: {len(balanced_df)} samples ({balanced_df['success'].sum()} positive)")
        clf, features = train_classifier(balanced_df, MODEL_PATHS[0])
# grasp classifier model using Random Forest
import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import joblib

def load_dataset(csv_path):
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
    
    clf = RandomForestClassifier(random_state=42)
    clf.fit(X_train, Y_train)
    
    train_acc = accuracy_score(Y_train, clf.predict(X_train))
    val_acc = accuracy_score(Y_test, clf.predict(X_test))
    print(f"Train accuracy: {train_acc*100:.1f}%, Validation accuracy: {val_acc*100:.1f}%")
    
    if model_path:
        joblib.dump(clf, model_path)
        print(f"model saved to {model_path}")
        
    return clf, features

if __name__ == "__main__":
    import os
    base = os.path.dirname(__file__)
    csv_paths = [os.path.join(base, "grasp_dataset_cube.csv"),
                 os.path.join(base, "grasp_dataset_duck.csv")]
    csv_paths = [p for p in csv_paths if os.path.exists(p)]
    
    df = load_dataset(csv_paths)
    print(f"Balanced: {len(df)} samples")
    clf, features = train_classifier(df, os.path.join(base, "grasp_classifier.pkl"))
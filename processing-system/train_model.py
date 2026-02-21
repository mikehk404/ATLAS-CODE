# train_model.py
"""
Train RandomForest model for landslide risk.
Input: history.csv in same folder with columns:
 timestamp,node_id,temp,humidity,pres,soil,vib,ndvi_local,moisture_mean,landslide

Output:
 rf_landslide.joblib saved in same folder (contains {model, features})
"""
import os
import pandas as pd
import numpy as np
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import TimeSeriesSplit
from sklearn.metrics import mean_absolute_error
import joblib

BASE_DIR = os.path.dirname(__file__)
HISTORY_CSV = os.path.join(BASE_DIR, "history.csv")
OUT_MODEL = os.path.join(BASE_DIR, "rf_landslide.joblib")
RANDOM_STATE = 42

if not os.path.exists(HISTORY_CSV):
    raise SystemExit("history.csv not found. Please put a history.csv with required columns in the folder.")

print("Loading history from", HISTORY_CSV)
df = pd.read_csv(HISTORY_CSV, parse_dates=["timestamp"])
df = df.sort_values(["node_id", "timestamp"]).reset_index(drop=True)

required = ["temp","humidity","pres","soil","vib","ndvi_local","moisture_mean","landslide"]
for c in required:
    if c not in df.columns:
        raise SystemExit(f"history.csv missing required column: {c}")

# Feature engineering: create lags and rolling features (grouped per node)
def make_features(df_in):
    df = df_in.copy()
    # per-node shifts
    for lag in (1,2,3):
        df[f"landslide_lag_{lag}"] = df.groupby("node_id")["landslide"].shift(lag)
        df[f"moisture_lag_{lag}"] = df.groupby("node_id")["moisture_mean"].shift(lag)
    # rolling means
    for r in (3,7):
        df[f"moisture_roll_{r}"] = df.groupby("node_id")["moisture_mean"].rolling(window=r, min_periods=1).mean().reset_index(0,drop=True)
        df[f"ndvi_roll_{r}"] = df.groupby("node_id")["ndvi_local"].rolling(window=r, min_periods=1).mean().reset_index(0,drop=True)
    # time-based features
    df["hour"] = pd.to_datetime(df["timestamp"]).dt.hour
    df["dayofyear"] = pd.to_datetime(df["timestamp"]).dt.dayofyear
    return df

df_feat = make_features(df)
# drop rows without target
df_feat = df_feat.dropna(subset=["landslide"])
# define features to use
candidate_feats = [
    "temp","humidity","pres","soil","vib","ndvi_local","moisture_mean",
    "landslide_lag_1","landslide_lag_2","landslide_lag_3",
    "moisture_lag_1","moisture_lag_2","moisture_lag_3",
    "moisture_roll_3","moisture_roll_7","ndvi_roll_3","ndvi_roll_7",
    "hour","dayofyear"
]
feat_cols = [c for c in candidate_feats if c in df_feat.columns]

X = df_feat[feat_cols].fillna(df_feat[feat_cols].median())
y = df_feat["landslide"].values

print("Training data shape:", X.shape)
if X.shape[0] < 20:
    print("Warning: small dataset for training:", X.shape[0])

# TimeSeriesSplit CV for validation
tscv = TimeSeriesSplit(n_splits=4)
maes = []
best_model = None
best_mae = 1e9

for fold, (train_idx, test_idx) in enumerate(tscv.split(X)):
    X_train, X_test = X.iloc[train_idx], X.iloc[test_idx]
    y_train, y_test = y[train_idx], y[test_idx]
    model = RandomForestRegressor(n_estimators=200, random_state=RANDOM_STATE, n_jobs=-1)
    model.fit(X_train, y_train)
    pred = model.predict(X_test)
    mae = mean_absolute_error(y_test, pred)
    maes.append(mae)
    print(f"Fold {fold} MAE: {mae:.4f}")
    if mae < best_mae:
        best_mae = mae
        best_model = model

print("CV mean MAE:", float(np.mean(maes)))
# Save best model and feature list
joblib.dump({"model": best_model, "features": feat_cols}, OUT_MODEL)
print("Saved model to", OUT_MODEL)
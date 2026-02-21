# predict_inference.py
import os
import joblib
import pandas as pd
from datetime import datetime

MODEL_PATH = os.path.join(os.path.dirname(__file__), "rf_landslide.joblib")

# try to load model
_model = None
_feat_cols = None
try:
    obj = joblib.load(MODEL_PATH)
    _model = obj.get("model")
    _feat_cols = obj.get("features", [])
    print("[predict_inference] Loaded model:", MODEL_PATH)
except Exception as e:
    print("[predict_inference] No model loaded (rf_landslide.joblib missing or invalid).", e)

def _make_feature_row(node, now_ts=None):
    """
    node: dict containing keys similar to map nodes:
      temp, humidity, pressure (or pres), soil or moisture, vibration, ndvi_local, moisture_mean
    returns: pandas.DataFrame with single row and columns matching _feat_cols
    """
    if now_ts is None:
        now_ts = datetime.utcnow()
    base = {}
    # normalize keys
    base["temp"] = node.get("temp", node.get("temperature", 25.0))
    base["humidity"] = node.get("humidity", 50.0)
    base["pres"] = node.get("pressure", node.get("pres", 50.0))
    base["soil"] = node.get("soil", node.get("moisture_mean", np_fallback(node)))
    base["vib"] = node.get("vibration", 0.0)
    base["ndvi_local"] = node.get("ndvi_local", 0.5)
    base["moisture_mean"] = node.get("moisture_mean", np_fallback(node))
    base["hour"] = now_ts.hour
    base["dayofyear"] = now_ts.timetuple().tm_yday

    # ensure all feat cols present (missing -> 0)
    row = {c: base.get(c, 0.0) for c in (_feat_cols or [])}
    return pd.DataFrame([row], columns=(_feat_cols or []))

def np_fallback(node):
    # fallback for moisture/soil if not provided
    try:
        if "moisture" in node and isinstance(node["moisture"], (list, tuple)):
            return float(sum(node["moisture"]) / len(node["moisture"]))
    except Exception:
        pass
    return 0.5

def predict_node_risk(node):
    """
    Return predicted landslide risk (float 0..1) or None if model missing.
    """
    if _model is None or not _feat_cols:
        return None
    X = _make_feature_row(node)
    try:
        pred = _model.predict(X)[0]
        # clamp 0..1
        return float(max(0.0, min(1.0, float(pred))))
    except Exception as e:
        print("[predict_inference] prediction error:", e)
        return None

# helper to predict series (if you have a history dataframe)
def predict_series_from_history(df_history):
    """
    df_history: pandas.DataFrame with columns matching features (and 'timestamp').
    returns: predicted numpy array aligned to df_history rows (or None if no model)
    """
    if _model is None or not _feat_cols:
        return None
    X = df_history.reindex(columns=_feat_cols).fillna(0.0)
    preds = _model.predict(X)
    preds = [float(max(0.0, min(1.0, float(p)))) for p in preds]
    return preds
# generate_history.py
import csv, random, os
from datetime import datetime, timedelta
BASE = os.path.dirname(__file__)
OUT = os.path.join(BASE, "history.csv")

start = datetime.now() - timedelta(days=90)
rows = []
for i in range(90):
    d = start + timedelta(days=i)
    for node in range(1,13):
        temp = random.uniform(24, 35)
        hum = random.uniform(50, 90)
        pres = random.uniform(30, 60)
        soil = random.uniform(10, 90)
        vib = random.uniform(0.01, 0.5)
        ndvi = random.uniform(0.2, 0.9)
        moisture_mean = soil/100.0
        # synthetic landslide label (simple function)
        landslide = min(max((moisture_mean*0.6) + (1-ndvi)*0.2 + (vib*0.4), 0.0), 1.0)
        rows.append([d.isoformat(), node, temp, hum, pres, soil, vib, ndvi, moisture_mean, landslide])

with open(OUT, "w", newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["timestamp","node_id","temp","humidity","pres","soil","vib","ndvi_local","moisture_mean","landslide"])
    writer.writerows(rows)
print("Generated", OUT)
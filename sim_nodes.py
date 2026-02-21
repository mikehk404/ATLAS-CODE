# sim_nodes.py
# Generate exc.txt sample with 12 REL_GNS nodes (format expected by Folium.py)
import random, os
from datetime import datetime

BASE = os.path.dirname(__file__)
OUT = os.path.join(BASE, "exc.txt")

# choose center somewhere in VN (configurable)
center_lat = 20.5   # adjust if you want different cluster
center_lon = 105.8

NUM_NODES = 12
rand = random.Random(42)

lines = []

# add a TM-only line first (sat telemetry)
volt = round(rand.uniform(3.7, 4.2), 2)
tm_line = f"TM:{volt},{center_lat:.6f},{center_lon:.6f},{rand.uniform(500000,510000):.2f},OK"
lines.append(tm_line)

# add one line containing multiple REL_GNS entries (all GN entries in one packet)
# Format per GN: GN,<id>,<TYPE>,<temp>,<hum>,<pres>,<soil>,<vib>,<lat>,<lon>
rel_gns_parts = []
for i in range(1, NUM_NODES+1):
    lat = center_lat + rand.uniform(-0.05, 0.05)
    lon = center_lon + rand.uniform(-0.05, 0.05)
    typ = "LANDSLIDE" if rand.random() < 0.5 else "NORMAL"
    temp = round(rand.uniform(24.0, 34.0), 1)
    hum = round(rand.uniform(50.0, 90.0), 1)
    pres = round(rand.uniform(30.0, 60.0), 1)
    soil = round(rand.uniform(10.0, 90.0), 2)
    vib = round(rand.uniform(0.01, 1.5), 2)
    part = f"GN,{i},{typ},{temp},{hum},{pres},{soil},{vib},{lat:.6f},{lon:.6f}"
    rel_gns_parts.append(part)

rel_payload = "REL_GNS:" + ";".join(rel_gns_parts)
# optionally add multispectral reference and bbox
ms_img = "ndvi_demo.png"   # if you have an image file, put its name; else keep but map will skip if image missing
bbox = f"BBOX:{center_lat-0.08:.6f},{center_lat+0.08:.6f},{center_lon-0.08:.6f},{center_lon+0.08:.6f}"
# compose second line
lines.append(tm_line + ";" + rel_payload + ";MS_IMG:" + ms_img + ";" + bbox)

# write file
with open(OUT, "w", encoding="utf-8") as f:
    for l in lines:
        f.write(l + "\n")

print(f"Wrote {OUT} with {NUM_NODES} REL_GNS nodes.")
print("Sample lines:")
for ln in lines:
    print(" ", ln)
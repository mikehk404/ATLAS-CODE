#PROJECT: ASCEND_ATLAS
#VERSION: 5

# folium_multi_risk_with_flood_ui_fix.py
import folium
from folium.plugins import HeatMap
import random
import numpy as np
import webbrowser
import os
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.dates import ConciseDateFormatter, AutoDateLocator
from datetime import datetime, timedelta
import io
import base64
import json

# --- Node load / simulate (same as before) ---
nodes = []
file_path = "nodes_data.txt"
if os.path.exists(file_path):
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                parts = line.strip().split(",")
                if len(parts) >= 11 and parts[0] == "NODE":
                    node_id = int(parts[1])
                    moisture = [float(parts[2]), float(parts[3]), float(parts[4])]
                    pressure = float(parts[5])
                    vibration = float(parts[6])
                    temp = float(parts[7])
                    humidity = float(parts[8])
                    lat = float(parts[9])
                    lon = float(parts[10])
                    nodes.append({"id": node_id, "moisture": moisture, "pressure": pressure, "vibration": vibration,
                                  "temp": temp, "humidity": humidity, "lat": lat, "lon": lon})
    except Exception as e:
        print(f"Error reading file: {e} - falling back to simulation")

if len(nodes) == 0:
    center_lat = random.uniform(16.0, 23.0)
    center_lon = random.uniform(102.0, 109.0)
    for i in range(12):
        lat = center_lat + random.uniform(-0.05, 0.05)
        lon = center_lon + random.uniform(-0.05, 0.05)
        moisture = [random.uniform(0.2, 0.6), random.uniform(0.18, 0.55), random.uniform(0.15, 0.5)]
        pressure = random.uniform(20, 60)
        vibration = random.uniform(0.05, 0.4)
        temp = random.uniform(25, 35)
        humidity = random.uniform(50, 90)
        nodes.append({"id": i+1, "moisture": moisture, "pressure": pressure, "vibration": vibration,
                      "temp": temp, "humidity": humidity, "lat": lat, "lon": lon})

# Sim satellite + hist
random.seed(42)
gnss_r = random.uniform(0.4, 0.9)
hist_ndvi_avg = random.uniform(0.3, 0.8)
pressure_trend = random.uniform(-0.3, 0.3)
rain_flag = random.choice([True, False])

# Risk functions (unchanged)
def calculate_landslide_risk(moisture, pressure, gnss_r, vibration, ndvi_hist_avg, pressure_trend):
    saturation = (np.mean(moisture) * 0.7) + (gnss_r * 0.3)
    p_norm = min(pressure / 80.0, 1.0)
    score = saturation + p_norm + vibration - 0.1
    if ndvi_hist_avg > 0.5:
        score -= 0.2
    score += pressure_trend * 0.15
    return min(max(score, 0.0), 1.0)

def calculate_wildfire_risk(temp, humidity, ndvi):
    return min(max((temp - 25)/15 * 0.5 + (1 - humidity/100) * 0.3 + (1 - ndvi) * 0.2, 0.0), 1.0)

def calculate_flood_risk(moisture, pressure_trend, rain_flag):
    return min(max(np.mean(moisture) * 0.5 + abs(pressure_trend if pressure_trend < 0 else 0) * 0.3 + (1 if rain_flag else 0) * 0.2, 0.0), 1.0)

# Create map
center_lat = np.mean([n["lat"] for n in nodes]) if nodes else 20.0
center_lon = np.mean([n["lon"] for n in nodes]) if nodes else 106.0
m = folium.Map(location=[center_lat, center_lon], zoom_start=10, tiles='OpenStreetMap')

# Expose JS map object as window.map for reliable JS access
map_js_name = m.get_name()  # Folium internal map variable name
m.get_root().html.add_child(folium.Element(f"<script>window.map = window.{map_js_name};</script>"))

# Wildfire heatmap
heat_data = [[n["lat"], n["lon"], calculate_wildfire_risk(n["temp"], n["humidity"], hist_ndvi_avg)] for n in nodes]
HeatMap(heat_data, radius=30, blur=15, gradient={0.0: 'green', 0.4: 'lime', 0.6: 'yellow', 0.8: 'orange', 1.0: 'red'}).add_to(m)

# Per-node radial areas (unchanged)
min_radius_m = 300
max_radius_m = 3500

for n in nodes:
    landslide_r = calculate_landslide_risk(n["moisture"], n["pressure"], gnss_r, n["vibration"], hist_ndvi_avg, pressure_trend)
    flood_r = calculate_flood_risk(n["moisture"], pressure_trend, rain_flag)

    if flood_r > 0.02:
        rad_m = int(min_radius_m + flood_r * (max_radius_m - min_radius_m))
        folium.Circle(
            location=[n["lat"], n["lon"]],
            radius=rad_m,
            color="darkblue",
            weight=1,
            fill=True,
            fill_color="blue",
            fill_opacity=0.14,
            tooltip=f"Node {n['id']} flood risk={flood_r:.2f}"
        ).add_to(m)

    if landslide_r > 0.02:
        rad_m = int(min_radius_m + landslide_r * (max_radius_m - min_radius_m))
        folium.Circle(
            location=[n["lat"], n["lon"]],
            radius=rad_m,
            color="darkred",
            weight=1,
            fill=True,
            fill_color="red",
            fill_opacity=0.18,
            tooltip=f"Node {n['id']} landslide risk={landslide_r:.2f}"
        ).add_to(m)

# Optional threshold hulls (same)
thresholds = [0.6, 0.7, 0.8]
flood_fill_opac = {0.6: 0.12, 0.7: 0.18, 0.8: 0.26}
land_fill_opac = {0.6: 0.12, 0.7: 0.18, 0.8: 0.26}

def build_hull_geo_coords_latlon(points_latlon):
    if len(points_latlon) < 3:
        return None
    arr = np.array([[p[1], p[0]] for p in points_latlon])
    try:
        hull = ConvexHull(arr)
        hull_coords = [list(arr[v]) for v in hull.vertices]
        if hull_coords[0] != hull_coords[-1]:
            hull_coords.append(hull_coords[0])
        return [[c for c in hull_coords]]
    except Exception:
        coords = [list(x) for x in arr]
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        return [coords]

for t in thresholds:
    pts = [[n["lat"], n["lon"]] for n in nodes if calculate_flood_risk(n["moisture"], pressure_trend, rain_flag) >= t]
    if len(pts) > 2:
        try:
            arr = np.array([[p[1], p[0]] for p in pts])
            hull = ConvexHull(arr)
            hull_coords = [ [arr[v][1], arr[v][0]] for v in hull.vertices ]
            hull_coords.append(hull_coords[0])
            folium.Polygon(hull_coords, color="darkblue", weight=2, fill=True, fill_color="blue", fill_opacity=flood_fill_opac.get(t, 0.15), tooltip=f"Flood hull >= {t}").add_to(m)
        except Exception:
            folium.Polygon(pts, color="darkblue", weight=2, fill=True, fill_color="blue", fill_opacity=flood_fill_opac.get(t, 0.15), tooltip=f"Flood hull >= {t}").add_to(m)

for t in thresholds:
    pts = [[n["lat"], n["lon"]] for n in nodes if calculate_landslide_risk(n["moisture"], n["pressure"], gnss_r, n["vibration"], hist_ndvi_avg, pressure_trend) >= t]
    if len(pts) > 2:
        try:
            arr = np.array([[p[1], p[0]] for p in pts])
            hull = ConvexHull(arr)
            hull_coords = [ [arr[v][1], arr[v][0]] for v in hull.vertices ]
            hull_coords.append(hull_coords[0])
            folium.Polygon(hull_coords, color="darkred", weight=2, fill=True, fill_color="red", fill_opacity=land_fill_opac.get(t, 0.15), tooltip=f"Landslide hull >= {t}").add_to(m)
        except Exception:
            folium.Polygon(pts, color="darkred", weight=2, fill=True, fill_color="red", fill_opacity=land_fill_opac.get(t, 0.15), tooltip=f"Landslide hull >= {t}").add_to(m)

# Node markers + nodes_for_js (unchanged except popup directions link)
nodes_for_js = []
for n in nodes:
    landslide_n = calculate_landslide_risk(n["moisture"], n["pressure"], gnss_r, n["vibration"], hist_ndvi_avg, pressure_trend)
    flood_n = calculate_flood_risk(n["moisture"], pressure_trend, rain_flag)
    wildfire_n = calculate_wildfire_risk(n["temp"], n["humidity"], hist_ndvi_avg)

    if landslide_n >= 0.6 and flood_n >= 0.6:
        marker_color = 'purple'
    else:
        dominant = max(('landslide', landslide_n), ('flood', flood_n), ('wildfire', wildfire_n), key=lambda x: x[1])
        if dominant[0] == 'landslide':
            marker_color = 'green' if dominant[1] < 0.6 else 'orange' if dominant[1] < 0.8 else 'red'
        elif dominant[0] == 'flood':
            marker_color = 'lightblue' if dominant[1] < 0.6 else 'blue'
        else:
            marker_color = 'green' if dominant[1] < 0.6 else 'orange' if dominant[1] < 0.8 else 'red'

    def safety_advice(landslide_r, flood_r):
        adv = []
        if landslide_r >= 0.6:
            adv.append("High landslide risk: move uphill, away from slopes/unstable ground; do not shelter under cliffs/trees; follow evacuation routes.")
        elif landslide_r >= 0.3:
            adv.append("Moderate landslide risk: avoid steep slopes and watch for signs (cracks, leaning trees).")
        if flood_r >= 0.6:
            adv.append("High flood risk: move to higher ground or upper floors; avoid rivers, bridges and low-lying roads.")
        elif flood_r >= 0.3:
            adv.append("Moderate flood risk: stay away from drainage channels and prepare to move to safer ground.")
        if not adv:
            adv.append("Low immediate hazard: stay alert and monitor updates.")
        return "<br>".join(adv)

    advice_html = safety_advice(landslide_n, flood_n)
    directions_link = f"https://www.google.com/maps/dir/?api=1&destination={n['lat']},{n['lon']}"
    popup_html = (f"<b>Node {n['id']}</b><br>"
                  f"Landslide: {landslide_n:.2f}<br>"
                  f"Flood: {flood_n:.2f}<br>"
                  f"Wildfire: {wildfire_n:.2f}<br><hr>"
                  f"{advice_html}<br><a href='{directions_link}' target='_blank'>Open directions (Google Maps)</a>")

    folium.CircleMarker(
        location=[n["lat"], n["lon"]],
        radius=8,
        color='black',
        fill=True,
        fill_color=marker_color,
        fill_opacity=0.95,
        popup=folium.Popup(popup_html, max_width=350)
    ).add_to(m)

    nodes_for_js.append({
        "id": n["id"],
        "lat": n["lat"],
        "lon": n["lon"],
        "landslide": round(landslide_n, 3),
        "flood": round(flood_n, 3),
        "wildfire": round(wildfire_n, 3),
        "advice": advice_html.replace('\n', '<br>')
    })

# Prediction graph (compact date labels)
days_past = 30
days_future = 7
dates_past = [datetime.now() - timedelta(days=i) for i in range(days_past, 0, -1)]
dates_past = list(reversed(dates_past))
dates_future = [datetime.now() + timedelta(days=i) for i in range(1, days_future + 1)]

random.seed(42)
historical_actual = [random.uniform(0.3, 0.7) + (i * 0.01) for i in range(days_past)]
historical_actual = np.array(historical_actual) + np.random.normal(0, 0.05, days_past)
historical_actual = np.clip(historical_actual, 0.0, 1.0)

historical_predicted = historical_actual.copy()
historical_predicted[1:] = historical_actual[:-1] + np.random.normal(0, 0.08, days_past-1)
historical_predicted[0] = 0.5
historical_predicted = np.clip(historical_predicted, 0.0, 1.0)

recent_trend = np.polyfit(range(-9, 1), historical_actual[-10:], 1)[0]
future_pred = [historical_actual[-1] + recent_trend * (i+1) + random.uniform(-0.05, 0.05) for i in range(days_future)]
future_pred = np.clip(future_pred, 0.0, 1.0)

fig, ax = plt.subplots(figsize=(8, 3.6))
ax.plot(dates_past, historical_actual, 'b-', label='Actual Risk (Past 30 Days)', linewidth=1.8)
ax.plot(dates_past, historical_predicted, color='orange', linestyle='--', label='Predicted (Model)', linewidth=1.6)
ax.plot(dates_future, future_pred, 'r:', label='Forecast (Next 7 days)', linewidth=1.8)

ax.set_xlabel('Date', fontsize=9)
ax.set_ylabel('Risk (0-1)', fontsize=9)
ax.set_title('Landslide Risk: Past 30 Days + 7-Day Forecast', fontsize=10)
ax.set_ylim(0,1); ax.grid(True, alpha=0.28)

locator = AutoDateLocator()
formatter = ConciseDateFormatter(locator)
ax.xaxis.set_major_locator(locator)
ax.xaxis.set_major_formatter(formatter)
plt.setp(ax.get_xticklabels(), rotation=25, ha='right', fontsize=7)

ax.legend(fontsize=8, loc='upper left')
plt.tight_layout()

buf = io.BytesIO()
plt.savefig(buf, format='png', dpi=140, bbox_inches='tight')
buf.seek(0)
img_base64 = base64.b64encode(buf.getvalue()).decode('ascii')
buf.close()
plt.close(fig)

img_html = f'<div style="text-align:center; margin-top:8px;"><h4 style="margin:6px 0 4px 0">Prediction Model (Landslide)</h4><img src="data:image/png;base64,{img_base64}" style="max-width:760px; width:85%; border:1px solid #444;"/></div>'
m.get_root().html.add_child(folium.Element(img_html))

# UI control (wrapped with DOMContentLoaded, reliable map access + highlight)
nodes_json = json.dumps(nodes_for_js)

ui_html = """
<div id="ascend-ui" style="position: fixed; left: 10px; top: 10px; z-index:1000; background: white; padding:8px; border-radius:6px;
    box-shadow: 0 2px 6px rgba(0,0,0,0.3); font-size:13px; width:260px;">
  <b>Nearest Node Finder</b><br>
  <select id="node-select" style="width:100%; margin-top:6px; margin-bottom:6px;">
    <option value="">-- Select node --</option>
  </select>
  <button id="btn-show" style="width:100%;">Show selected node</button>
  <button id="btn-myloc" style="width:100%; margin-top:6px;">Use my location (browser)</button>
  <hr style="margin:6px 0;">
  <a href="#" id="toggle-list" style="font-size:12px;">Show / Hide nodes list</a>
  <div id="nodes-list" style="display:none; max-height:260px; overflow:auto; margin-top:6px; font-size:12px;">
  </div>
  <div style="font-size:11px; margin-top:6px; color:#333;">Tip: allow browser location to find nearest node. Directions open in Google Maps.</div>
</div>

<script>
document.addEventListener('DOMContentLoaded', function(){
    try {
        var nodes = """ + nodes_json + """;
        var select = document.getElementById('node-select');
        var listDiv = document.getElementById('nodes-list');

        // populate dropdown and list
        nodes.forEach(function(n){
            var opt = document.createElement('option');
            opt.value = n.id;
            opt.text = 'Node ' + n.id + ' (L:' + n.landslide.toFixed(2) + ' F:' + n.flood.toFixed(2) + ')';
            select.appendChild(opt);

            var item = document.createElement('div');
            item.innerHTML = '<b>Node ' + n.id + '</b> - L:' + n.landslide + ' F:' + n.flood + '<br><small>' + n.advice + '</small><hr>';
            listDiv.appendChild(item);
        });

        // haversine
        function toRad(x){return x*Math.PI/180;}
        function haversine(lat1, lon1, lat2, lon2){
            var R = 6371.0;
            var dLat = toRad(lat2-lat1), dLon = toRad(lon2-lon1);
            var a = Math.sin(dLat/2)*Math.sin(dLat/2) + Math.cos(toRad(lat1))*Math.cos(toRad(lat2))*Math.sin(dLon/2)*Math.sin(dLon/2);
            var c = 2*Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
            return R * c;
        }

        function findNearest(lat, lon){
            var best = null; var bestd = 1e12;
            nodes.forEach(function(n){
                var d = haversine(lat, lon, n.lat, n.lon);
                if (d < bestd){ bestd = d; best = n; }
            });
            return {node: best, distance_km: bestd};
        }

        // find map reliably
        var map = window.map || null;
        if(!map){
            for(var k in window){
                try{
                    var cand = window[k];
                    if(cand && typeof cand.setView === 'function' && cand._layers){
                        map = cand; window.map = cand; break;
                    }
                }catch(e){}
            }
        }

        if(!map){
            console.warn('Leaflet map object not found. Node finder will be limited.');
        }

        // highlight layer
        var highlightLayer = null;
        function clearHighlight(){
            try{ if(highlightLayer && map){ map.removeLayer(highlightLayer); } }catch(e){}
            highlightLayer = null;
        }
        function highlightNode(n){
            if(!map) return;
            clearHighlight();
            highlightLayer = L.circleMarker([n.lat, n.lon], {radius:14, color:'#ffff66', weight:3, fill:false, pane: 'overlayPane'}).addTo(map);
            map.setView([n.lat, n.lon], 14);
            // open popup with directions link
            var dirUrl = 'https://www.google.com/maps/dir/?api=1&destination=' + n.lat + ',' + n.lon;
            var popupHtml = '<b>Node ' + n.id + '</b><br>Landslide: ' + n.landslide + '<br>Flood: ' + n.flood + '<br>Wildfire: ' + n.wildfire + '<hr>' + n.advice + '<br><a href="' + dirUrl + '" target="_blank">Open directions (Google Maps)</a>';
            L.popup({maxWidth:350}).setLatLng([n.lat, n.lon]).setContent(popupHtml).openOn(map);
        }

        // show node handler
        function showNodeById(id){
            var n = nodes.find(function(x){ return x.id == id; });
            if(!n){ alert('Node not found'); return; }
            highlightNode(n);
        }

        document.getElementById('btn-show').addEventListener('click', function(){
            var id = select.value;
            if(!id){ alert('Choose a node first or use your location'); return; }
            showNodeById(id);
        });

        document.getElementById('btn-myloc').addEventListener('click', function(){
            if (!navigator.geolocation){ alert('Geolocation not available in this browser'); return; }
            navigator.geolocation.getCurrentPosition(function(pos){
                var lat = pos.coords.latitude, lon = pos.coords.longitude;
                var res = findNearest(lat, lon);
                if(res.node){
                    highlightNode(res.node);
                    // open Google Maps directions with origin set
                    var dirUrl = 'https://www.google.com/maps/dir/?api=1&origin=' + lat + ',' + lon + '&destination=' + res.node.lat + ',' + res.node.lon;
                    window.open(dirUrl, '_blank');
                } else {
                    alert('No nodes found.');
                }
            }, function(err){
                alert('Geolocation failed: ' + err.message);
            });
        });

        document.getElementById('toggle-list').addEventListener('click', function(e){
            e.preventDefault();
            listDiv.style.display = (listDiv.style.display === 'none') ? 'block' : 'none';
        });

        // cleanup highlight if map clicked elsewhere
        if(map){
            map.on('click', function(){ clearHighlight(); });
        }

        console.log('ASCEND UI initialized.');
    } catch(err){
        console.error('ASCEND UI init error:', err);
        alert('Node finder initialization error. See console for details.');
    }
});
</script>
"""

m.get_root().html.add_child(folium.Element(ui_html))

# Title & notes (unchanged)
title_html = '<h3 align="center" style="font-size:20px"><b>ASCEND Multi-Disaster Risk Map (landsld/flood/wildfire)</b></h3>'
m.get_root().html.add_child(folium.Element(title_html))
notes_html = '<p align="center" style="font-size:12px">Notes: Red=landslide radii, Blue=flood radii, Purple=node both high. Heatmap=wildfire. Directions open Google Maps.</p>'
m.get_root().html.add_child(folium.Element(notes_html))

map_file = 'ascend_multi_risk_map_with_flood_and_ui_fixed2.html'
m.save(map_file)
print(f"Interactive map exported: {map_file}")
webbrowser.open('file://' + os.path.realpath(map_file))


#test
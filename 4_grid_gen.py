import folium
from branca.element import Template, MacroElement
import numpy as np
from shapely.geometry import Point

# You need Google Maps API key to display the map, otherwise you can use OpenStreetMap tiles (line 56).
api_key = "YOUR_API_KEY"

# Define the locations
airstrip_center = (36.9023389, 0.0213972)
restricted_radius_km = 1
camp = (36.8688194, 0.0248028)
camera_position = (36.964531, 0.024991)

# Define grid resolution (approx. 100m per grid cell)
resolution = 0.001  # ~111m per degree of latitude

# Define grid bounds (expand around the area of interest)
min_lat = min(camp[1], camera_position[1], airstrip_center[1]) - 0.01
max_lat = max(camp[1], camera_position[1], airstrip_center[1]) + 0.01
min_lon = min(camp[0], camera_position[0], airstrip_center[0]) - 0.01
max_lon = max(camp[0], camera_position[0], airstrip_center[0]) + 0.01

# Create grid
lat_steps = int((max_lat - min_lat) / resolution) + 1
lon_steps = int((max_lon - min_lon) / resolution) + 1
grid = np.zeros((lat_steps, lon_steps))  # 0 = free, 1 = obstacle

# Mark restricted zone as obstacles
for i in range(lat_steps):
    for j in range(lon_steps):
        lat = min_lat + i * resolution
        lon = min_lon + j * resolution
        point = Point(lon, lat)
        # Convert radius to degrees
        if point.distance(Point(airstrip_center)) <= restricted_radius_km / 111:
            grid[i, j] = 1  # Mark as obstacle

# Convert grid to lat/lon points for visualization
grid_points = []
for i in range(lat_steps):
    for j in range(lon_steps):
        lat = min_lat + i * resolution
        lon = min_lon + j * resolution
        grid_points.append((lat, lon, grid[i, j]))

# Create a map with Google Maps tiles
m = folium.Map(
    location=[(min_lat + max_lat) / 2, (min_lon + max_lon) / 2],
    zoom_start=13,
    tiles=f"https://maps.googleapis.com/maps/vt?lyrs=m&x={{x}}&y={{y}}&z={{z}}&key={api_key}",
    attr="Google Maps"
)

# Use this instead to use OpenStreetMap tiles
# m = folium.Map(location=[(min_lat + max_lat) / 2, (min_lon + max_lon) / 2], zoom_start=13)

# Add the grid points to the map
for lat, lon, value in grid_points:
    if value == 1:  # Obstacle cells
        folium.CircleMarker(
            location=[lat, lon],
            radius=1,
            color="black",
            fill=True,
            fill_opacity=0.7
        ).add_to(m)
    else:  # Free cells (optional for visualization clarity)
        folium.CircleMarker(
            location=[lat, lon],
            radius=0.5,
            color="lightgray",
            fill=True,
            fill_opacity=0.2
        ).add_to(m)

# Add the restricted zone as a circle
folium.Circle(
    location=[airstrip_center[1], airstrip_center[0]],
    radius=restricted_radius_km * 1000,  # Convert km to meters
    color="red",
    fill=True,
    fill_opacity=0.3,
    tooltip="Restricted Zone (2 km radius)"
).add_to(m)

# Add the camp and camera placement as markers
folium.Marker(
    location=[camp[1], camp[0]],
    popup="Camp",
    icon=folium.Icon(color="blue", icon="home")
).add_to(m)

folium.Marker(
    location=[camera_position[1], camera_position[0]],
    popup="Camera Placement",
    icon=folium.Icon(color="green", icon="camera")
).add_to(m)

# Add a legend to the map
legend_html = """
{% macro html(this, kwargs) %}
<div style="
    position: fixed; 
    bottom: 50px; left: 50px; width: 200px; height: 140px; 
    background-color: white; 
    border:2px solid grey; 
    z-index:9999; 
    font-size:14px;
    padding: 10px;
">
<b>Legend</b><br>
<span style="color:blue;">&#9679;</span> Camp (Start)<br>
<span style="color:green;">&#9679;</span> Camera Placement (Goal)<br>
<span style="color:red;">&#9679;</span> Airstrip Restricted Zone<br>
<span style="color:black;">&#9679;</span> Obstacle Cells<br>
<span style="color:lightgray;">&#9679;</span> Free Cells<br>
</div>
{% endmacro %}
"""
legend = MacroElement()
legend._template = Template(legend_html)
m.get_root().add_child(legend)

# Save the map as an HTML file
m.save("4_grid_with_restricted_zone_ol_pejeta.html")
print("Map saved as '4_grid_with_restricted_zone_ol_pejeta.html'.")

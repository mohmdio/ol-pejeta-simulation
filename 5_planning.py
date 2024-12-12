import heapq
import numpy as np
import folium
from shapely.geometry import Point
from branca.element import Template, MacroElement

# You need Google Maps API key to display the map, otherwise you can use OpenStreetMap tiles (line 122).
api_key = "YOUR_API_KEY"

# Define the locations
airstrip_center = (36.9023389, 0.0213972)
restricted_radius_km = 1
camp = (36.8688194, 0.0248028)
camera_position = (36.964531, 0.024991)

# Define grid resolution (approx. 100m per grid cell)
resolution = 0.001  # ~111m per degree of latitude

# Define grid bounds
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

# A* helper functions


def heuristic(a, b):
    """Compute the heuristic distance (Euclidean)."""
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)


def a_star_search(grid, start, goal):
    """A* pathfinding algorithm."""
    rows, cols = grid.shape
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]  # Up, down, left, right

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct the path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Return reversed path

        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] == 0:  # Free cell
                # Assume uniform cost for simplicity
                tentative_g_score = g_score[current] + 1
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + \
                        heuristic(neighbor, goal)
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found

# Convert lat/lon to grid indices


def latlon_to_grid(lat, lon, min_lat, min_lon, resolution):
    i = int((lat - min_lat) / resolution)
    j = int((lon - min_lon) / resolution)
    return i, j


def grid_to_latlon(i, j, min_lat, min_lon, resolution):
    lat = min_lat + i * resolution
    lon = min_lon + j * resolution
    return lat, lon


# Map camp and camera placement to grid indices
start_idx = latlon_to_grid(camp[1], camp[0], min_lat, min_lon, resolution)
goal_idx = latlon_to_grid(
    camera_position[1], camera_position[0], min_lat, min_lon, resolution)

# Run A* algorithm
path_indices = a_star_search(grid, start_idx, goal_idx)

# Convert path indices back to lat/lon for visualization
path_coordinates = [grid_to_latlon(
    i, j, min_lat, min_lon, resolution) for i, j in path_indices]

# Visualize on a Google Maps Tile-based map
m = folium.Map(
    location=[(min_lat + max_lat) / 2, (min_lon + max_lon) / 2],
    zoom_start=13,
    tiles=f"https://maps.googleapis.com/maps/vt?lyrs=m&x={{x}}&y={{y}}&z={{z}}&key={api_key}",
    attr="Google Maps"
)

# Use this instead to use OpenStreetMap tiles
# m = folium.Map(location=[(min_lat + max_lat) / 2, (min_lon + max_lon) / 2], zoom_start=13)

# Add the path to the map
folium.PolyLine(path_coordinates, color="blue", weight=2.5,
                tooltip="Optimal Path").add_to(m)

# Add the restricted zone as a circle
folium.Circle(
    location=[airstrip_center[1], airstrip_center[0]],
    radius=restricted_radius_km * 1000,  # Convert km to meters
    color="red",
    fill=True,
    fill_opacity=0.3,
    tooltip="Restricted Zone (1 km radius)"
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
<span style="color:blue;">&nbsp;&#9646;</span> Optimal Path<br>
</div>
{% endmacro %}
"""
legend = MacroElement()
legend._template = Template(legend_html)
m.get_root().add_child(legend)

# Save the map as an HTML file
m.save("5_a_star_path_planning_ol_pejeta.html")
print("Map saved as '5_a_star_path_planning_ol_pejeta.html'.")

# Print path coordinates in pair no letters, to be used in the next file as path_coordinates array
print("Path coordinates:")
for i, coord in enumerate(path_coordinates):
    print(f"{coord[0]},{coord[1]}")

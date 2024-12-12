import pandas as pd
import folium
from folium.plugins import HeatMap
import matplotlib.pyplot as plt
import seaborn as sns

# You need Google Maps API key to display the map, otherwise you can use OpenStreetMap tiles."
api_key = "YOUR_API_KEY"

#  Load the CSV file
df = pd.read_csv("eastern_rhino_locations.csv")

# Create a Folium Map
# Center the map around the average latitude and longitude
map_center = [df['latitude'].mean(), df['longitude'].mean()]
# Create the map with Google Maps tiles
m = folium.Map(
    location=map_center,
    zoom_start=14,
    tiles=f"https://maps.googleapis.com/maps/vt?lyrs=m&x={{x}}&y={{y}}&z={{z}}&key={api_key}",
    attr="Google Maps"
)
# Use this instead to use OpenStreetMap tiles
# m = folium.Map(location=map_center, zoom_start=14)

# Add the raw rhino locations as points on the map
for _, row in df.iterrows():
    folium.CircleMarker(
        location=[row['latitude'], row['longitude']],
        radius=3,
        color="red",
        fill=True,
        fill_opacity=0.7
    ).add_to(m)

# Add a heat map layer
heat_data = [[row['latitude'], row['longitude']] for _, row in df.iterrows()]
HeatMap(heat_data, radius=15).add_to(m)

# Save the interactive heatmap to an HTML file
m.save("2_eastern_rhino_heatmap.html")
print("Interactive heatmap saved as '2_eastern_rhino_heatmap.html'")

# Generate a static density heatmap using Seaborn
plt.figure(figsize=(10, 8))
sns.kdeplot(
    x=df['longitude'],
    y=df['latitude'],
    cmap="Reds",
    fill=True,
    bw_adjust=0.5
)
plt.title("Kernel Density Heatmap of Eastern Black Rhino Locations")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.show()

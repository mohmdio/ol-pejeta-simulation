import pandas as pd
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt

# Load Rhino Location Data
df = pd.read_csv("eastern_rhino_locations.csv")

# Extract latitude and longitude
coordinates = df[['latitude', 'longitude']]

# Perform K-Means Clustering
num_cameras = 2
kmeans = KMeans(n_clusters=num_cameras, random_state=42)
df['cluster'] = kmeans.fit_predict(coordinates)

# Get cluster centers (camera placements)
camera_positions = kmeans.cluster_centers_

# Print the latitude and longitude for each camera placement
for idx, (lat, lon) in enumerate(camera_positions, start=1):
    print(f"Camera Placement {idx}: Latitude {lat:.6f}, Longitude {lon:.6f}")

# Plot Clusters with Labels
plt.figure(figsize=(10, 8))
for cluster_id in range(num_cameras):
    cluster_points = df[df['cluster'] == cluster_id]
    plt.scatter(
        cluster_points['longitude'], cluster_points['latitude'],
        label=f"Cluster {cluster_id + 1} (Rhino Locations)"
    )

# Plot Camera Positions
plt.scatter(
    camera_positions[:, 1], camera_positions[:, 0],
    c='red', marker='X', s=200, label="Camera Placements (Cluster Centroids)"
)

plt.title("Optimal Camera Placements using K-Means Clustering")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.grid()
plt.show()

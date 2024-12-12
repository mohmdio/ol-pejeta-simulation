import random
import pandas as pd

# Define the bounding box (latitude and longitude coordinates)
bounding_box = {
    "min_lat": 0.0242136,  # Converted from 0° 1'27.17"N
    "max_lat": 0.025775,   # Converted from 0° 1'32.79"N
    "min_lon": 36.9632472, # Converted from 36°57'47.69"E
    "max_lon": 36.9649472  # Converted from 36°57'53.81"E
}

# Generate random points
num_points = 100
data = []

for _ in range(num_points):
    lat = random.uniform(bounding_box["min_lat"], bounding_box["max_lat"])
    lon = random.uniform(bounding_box["min_lon"], bounding_box["max_lon"])
    data.append({"latitude": lat, "longitude": lon})

# Convert to DataFrame and save as CSV
df = pd.DataFrame(data)
df.to_csv("eastern_rhino_locations.csv", index=False)

print("Generated rhino location points and saved as 'eastern_rhino_locations.csv'.")

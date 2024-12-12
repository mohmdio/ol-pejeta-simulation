# Simulation of Camera Placement and Recovery in Ol Pejeta Conservancy

This project simulates the drone's camera placement path to the Black Rhino Enclosure Viewpoint in the Ol Pejeta Conservancy. Using Python libraries, it visualises suggested placements with a kernel density heat map and K-clustering, plans an optimal path with the A* algorithm, and simulates the path via SITL with ADS-B for real-time traffic monitoring, adjusting altitude to avoid collisions.

## Requirements

This project is built using Python 3.8.10, and you need the following libraries to run the code:

- `random`
- `pandas`
- `folium`
- `matplotlib`
- `seaborn`
- `sklearn`
- `numpy`
- `shapely`
- `pymavlink`
- `geopy`

You can install all the required libraries using `pip` by running the following command:

```bash
pip install -r requirements.txt
```

# Command to run SITL with ADS-B simulation:
```bash
--home=0.0248028,36.8688194,100,90 --param SIM_ADSB_TX=1 --param SIM_ADSB_COUNT=3 --param SIM_ADSB_ALT=50 --param SIM_ADSB_RADIUS=5000 --param SIM_ADSB_TYPES=1
```

## License
This project is licensed under the terms of the **GNU General Public License (GPL)**, version 3.0. 

See the [LICENSE](./LICENSE) file for more details.
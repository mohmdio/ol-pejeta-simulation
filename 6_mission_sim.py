from pymavlink import mavutil
from geopy.distance import geodesic
import time
import logging

# Configure the logger
logging.basicConfig(
    # Set the logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    level=logging.DEBUG,
    format='%(asctime)s - %(levelname)s - %(message)s',  # Log message format
)

"""
Start the SITL simulation with the following command:
--home=0.0248028,36.8688194,100,90 --param SIM_ADSB_TX=1 --param SIM_ADSB_COUNT=3 --param SIM_ADSB_ALT=50 --param SIM_ADSB_RADIUS=5000 --param SIM_ADSB_TYPES=1
"""

# Connect to the vehicle
master = mavutil.mavlink_connection("tcp:127.0.0.1:14550")
master.wait_heartbeat()
logging.info("Heartbeat received. Connected to the vehicle.")

# Mission parameters
# Safe distance in meters for ADS-B conflicts (Radius)
safe_horizontal_distance = 5000
# Safe distance in meters for ADS-B conflicts (Altitude)
safe_vertical_distance = 1000
altitude = 100  # Altitude in meters
maximum_altitude = 120  # Maximum allowed altitude in meters
restricted_zones = [
    (0.0213972, 36.9023389, 1000)  # Airstrip center, radius in meters
]


def is_in_restricted_zone(lat, lon, restricted_zones):
    """
    Check if a waypoint is in a restricted zone
    """
    for zone in restricted_zones:
        center = (zone[0], zone[1])
        radius = zone[2]
        if geodesic((lat, lon), center).meters <= radius:
            return True
    return False


def set_param(master, param_name, param_value):
    """
    Set a parameter on the vehicle
    """
    try:
        logging.info(f"Setting {param_name} to {param_value}...")
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            float(param_value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        # Wait for ACK from the vehicle
        ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if ack and ack.param_id.strip('\x00') == param_name:
            logging.info(f"{param_name} set to {ack.param_value}")
        else:
            logging.error(f"Failed to set {param_name}.")
    except Exception as e:
        logging.error(f"Error setting {param_name}: {e}")


def send_command(command, confirmation, param1=0, param2=0, param3=0, param4=0, param5=0, param6=0, param7=0):
    """
    Send a COMMAND_LONG message to the vehicle
    """
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            command,
            confirmation,
            param1,
            param2,
            param3,
            param4,
            param5,
            param6,
            param7
        )
        logging.error(f"Command {command} sent successfully.")
    except Exception as e:
        logging.error(f"Error sending command {command}: {e}")


def configure_adsb(master):
    """
    Configure ADS-B and Avoidance parameters
    https://ardupilot.org/plane/docs/parameters.html
    """
    logging.info("Configuring ADS-B and Avoidance parameters...")
    set_param(master, "ADSB_TYPE", 1)        # MAVLink-based ADS-B
    set_param(master, "AVD_ENABLE", 0)       # Enable ADS-B avoidance
    set_param(master, "AVD_F_ACTION", 6)     # Hover on conflict
    set_param(master, "AVD_F_DIST_XY", 100)  # Minimum horizontal separation
    set_param(master, "AVD_F_DIST_Z", 100)   # Minimum vertical separation
    set_param(master, "AVD_F_RCVRY", 1)      # Resume previous mode
    set_param(master, "AVD_F_TIME", 30)      # Collision projection time


def add_mission_waypoint_int(seq, lat, lon, alt):
    """
    Add a waypoint to the mission list
    """
    try:
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 1, 0, 0, 0, 0,
            # Convert latitude to an integer in 1e7 degrees as required by MAVLink
            int(lat * 1e7),
            # Convert longitude to an integer in 1e7 degrees as required by MAVLink
            int(lon * 1e7),
            alt
        )
        logging.info(
            f"Waypoint {seq} added: Latitude {lat}, Longitude {lon}, Altitude {alt}")
    except Exception as e:
        logging.error(f"Error adding waypoint {seq}: {e}")


def clear_previous_mission(master):
    """
    Clears all previous waypoints and mission commands stored in the ardupilot.
    Note: This operation does not modify system parameters.
    """
    master.mav.mission_clear_all_send(
        master.target_system, master.target_component)

    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)

    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        logging.info("Mission parameters cleared successfully.")
    else:
        logging.error(
            "Failed to clear mission parameters. Check the connection or system state.")


def upload_mission(master, filtered_waypoints):
    """
    Upload the mission to the autopilot
    """
    clear_previous_mission(master)

    logging.info("Uploading new mission...")
    master.mav.mission_count_send(
        master.target_system, master.target_component, len(filtered_waypoints))

    for seq, wp in enumerate(filtered_waypoints):
        lat, lon, alt = wp
        logging.info(
            f"Adding waypoint {seq + 1}: Latitude {lat}, Longitude {lon}, Altitude {alt}")
        add_mission_waypoint_int(seq, lat, lon, alt)

    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=10)
    if ack:
        logging.info("Mission uploaded successfully.")
    else:
        logging.error("Mission upload failed. Please check connection.")


def listen_adsb(master):
    """
    Listen for ADS-B messages from other aircrafts
    """
    msg = master.recv_match(type='ADSB_VEHICLE', blocking=False, timeout=10)
    if msg:
        altitude_in_meters = msg.altitude / 1000.0  # Convert altitude to meters
        logging.info(
            f"ADS-B Aircraft Detected: ICAO={msg.ICAO_address}, Lat={msg.lat/1e7}, Lon={msg.lon/1e7}, Alt={altitude_in_meters}")
        return {
            'icao': msg.ICAO_address,
            'lat': msg.lat / 1e7,
            'lon': msg.lon / 1e7,
            'alt': altitude_in_meters
        }
    return None


def is_conflict(drone_position, adsb_data):
    """
    Check if there is a conflict between the drone and an ADS-B aircraft
    """
    horizontal_distance = geodesic((drone_position[0], drone_position[1]),
                                   (adsb_data['lat'], adsb_data['lon'])).meters
    # Altitude difference
    vertical_distance = abs(drone_position[2] - adsb_data['alt'])
    return horizontal_distance < safe_horizontal_distance and vertical_distance < safe_vertical_distance


# Dictionary to store ICAO IDs of resolved conflicts
resolved_conflicts = {}


def adjust_altitude(master, current_waypoints, adsb_data):
    """
    Adjust the altitude of the drone to avoid a conflict and store resolved conflicts. And ensure that the altitude does not exceed the legal limit (maximum_altitude).
    """

    # Check if the conflict with this ICAO has already been resolved
    icao_id = adsb_data['icao']
    if icao_id in resolved_conflicts:
        logging.info(
            f"Conflict with ICAO={icao_id} already resolved. Skipping adjustment.")
        return current_waypoints

    logging.warning("Adjusting altitude due to conflict...")
    adjusted_waypoints = []

    for wp in current_waypoints:
        # Check if the drone's altitude is below or above the conflict altitude
        if wp[2] <= adsb_data['alt']:
            # Decrease altitude by 20m but ensure it doesn't go below 0
            new_alt = wp[2] - 20
            if new_alt < 0:
                new_alt = 0
        else:
            # Increase altitude by 20m, but cap it to the max legal altitude of 120m
            new_alt = wp[2] + 20
            if new_alt > maximum_altitude: 
                new_alt = maximum_altitude

        logging.warning(f"Waypoint altitude adjusted to {new_alt} meters.")
        adjusted_waypoints.append((wp[0], wp[1], new_alt))

    # Mark the conflict as resolved by storing the ICAO ID
    resolved_conflicts[icao_id] = True
    logging.info(f"Conflict with ICAO={icao_id} resolved and stored.")

    # Upload the adjusted mission
    upload_mission(master, adjusted_waypoints)
    return adjusted_waypoints


def execute_mission(master, waypoints):
    """
    Execute the mission while monitoring for conflicts
    """
    def set_flight_mode(mode):
        """
        Set the flight mode of the vehicle
        """
        mode_id = master.mode_mapping(
        )[mode]  # Get the mode ID from the mode name
        master.mav.set_mode_send(
            master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    def arm_and_takeoff(target_altitude):
        """
        Arm the vehicle and takeoff to the target altitude
        """
        set_flight_mode("GUIDED")
        logging.info("Arming the vehicle...")
        master.arducopter_arm()
        master.motors_armed_wait()
        logging.info("Vehicle armed.")
        logging.info(f"Taking off to {target_altitude} meters...")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0, 0, 0, 0,
            0, 0, target_altitude
        )
        logging.info("Takeoff command sent. Monitoring altitude...")
        while True:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if msg:
                current_altitude = msg.relative_alt / 1000.0  # Convert mm to meters
                logging.info(f"Current Altitude: {current_altitude:.1f}m")
                if current_altitude >= target_altitude * 0.95:  # 95% of target altitude
                    logging.info("Reached target altitude.")
                    break

    arm_and_takeoff(altitude)
    set_flight_mode("AUTO")  # Switch to AUTO mode to start the mission
    logging.info("Executing mission...")

    while True:
        try:
            # Monitor the drone position
            msg = master.recv_match(
                type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
            if msg:
                drone_position = (msg.lat / 1e7, msg.lon / 1e7,
                                  msg.relative_alt / 1000.0)
                logging.info(f"Drone Position: {drone_position}")
                adsb_data = listen_adsb(master)

                """
                Multiple approaches can be taken if a conflict is detected:
                1. Hover in place until the conflict is resolved. (Loiter)
                2. Adjust the path to avoid the conflicting aircraft.
                3. Adjust the altitude to avoid the conflicting aircraft.
                """
                if adsb_data and is_conflict(drone_position, adsb_data):
                    logging.critical("Conflict detected! Changing altitude...")
                    waypoints = adjust_altitude(master, waypoints, adsb_data)
            time.sleep(1)
        # Reconnect if the connection is lost
        except ConnectionResetError:
            logging.critical("Connection lost. Attempting to reconnect...")
            master.close()
            time.sleep(5)
            master = mavutil.mavlink_connection("tcp:127.0.0.1:14550")
            master.wait_heartbeat()
            logging.info("Reconnected to the vehicle.")


# Filter waypoints for restricted zones
path_coordinates = [
    [0.024397, 36.867819],
    [0.024397, 36.868819],
    [0.024397, 36.869819],
    [0.024397, 36.870819],
    [0.024397, 36.871819],
    [0.024397, 36.872819],
    [0.024397, 36.873819],
    [0.024397, 36.874819],
    [0.024397, 36.875819],
    [0.024397, 36.876819],
    [0.024397, 36.877819],
    [0.024397, 36.878819],
    [0.024397, 36.879819],
    [0.024397, 36.880819],
    [0.024397, 36.881819],
    [0.024397, 36.882819],
    [0.024397, 36.883819],
    [0.024397, 36.884819],
    [0.024397, 36.885819],
    [0.024397, 36.886819],
    [0.024397, 36.887819],
    [0.024397, 36.888819],
    [0.024397, 36.889819],
    [0.024397, 36.890819],
    [0.024397, 36.891819],
    [0.024397, 36.892819],
    [0.024397, 36.893819],
    [0.025397, 36.893819],
    [0.026397, 36.893819],
    [0.026397, 36.894819],
    [0.027397, 36.894819],
    [0.028397, 36.894819],
    [0.028397, 36.895819],
    [0.029397, 36.895819],
    [0.029397, 36.896819],
    [0.029397, 36.897819],
    [0.030397, 36.897819],
    [0.030397, 36.898819],
    [0.030397, 36.899819],
    [0.030397, 36.900819],
    [0.030397, 36.901819],
    [0.030397, 36.902819],
    [0.030397, 36.903819],
    [0.030397, 36.904819],
    [0.030397, 36.905819],
    [0.030397, 36.906819],
    [0.030397, 36.907819],
    [0.030397, 36.908819],
    [0.030397, 36.909819],
    [0.030397, 36.910819],
    [0.030397, 36.911819],
    [0.030397, 36.912819],
    [0.030397, 36.913819],
    [0.030397, 36.914819],
    [0.030397, 36.915819],
    [0.030397, 36.916819],
    [0.030397, 36.917819],
    [0.030397, 36.918819],
    [0.030397, 36.919819],
    [0.030397, 36.920819],
    [0.030397, 36.921819],
    [0.030397, 36.922819],
    [0.030397, 36.923819],
    [0.030397, 36.924819],
    [0.030397, 36.925819],
    [0.030397, 36.926819],
    [0.030397, 36.927819],
    [0.030397, 36.928819],
    [0.030397, 36.929819],
    [0.030397, 36.930819],
    [0.030397, 36.931819],
    [0.030397, 36.932819],
    [0.030397, 36.933819],
    [0.030397, 36.934819],
    [0.030397, 36.935819],
    [0.030397, 36.936819],
    [0.030397, 36.937819],
    [0.030397, 36.938819],
    [0.030397, 36.939819],
    [0.030397, 36.940819],
    [0.030397, 36.941819],
    [0.030397, 36.942819],
    [0.030397, 36.943819],
    [0.030397, 36.944819],
    [0.030397, 36.945819],
    [0.030397, 36.946819],
    [0.030397, 36.947819],
    [0.030397, 36.948819],
    [0.030397, 36.949819],
    [0.030397, 36.950819],
    [0.030397, 36.951819],
    [0.030397, 36.952819],
    [0.030397, 36.953819],
    [0.030397, 36.954819],
    [0.030397, 36.955819],
    [0.030397, 36.956819],
    [0.030397, 36.957819],
    [0.029397, 36.957819],
    [0.029397, 36.958819],
    [0.028397, 36.958819],
    [0.028397, 36.959819],
    [0.027397, 36.959819],
    [0.027397, 36.960819],
    [0.026397, 36.960819],
    [0.026397, 36.961819],
    [0.025397, 36.961819],
    [0.025397, 36.962819],
    [0.024397, 36.962819],
    [0.024397, 36.963819]
]
waypoints = [(lat, lon, altitude)
             for lat, lon in path_coordinates]  # Add altitude to each waypoint
filtered_waypoints = [wp for wp in waypoints if not is_in_restricted_zone(
    wp[0], wp[1], restricted_zones)]  # Filter waypoints for restricted zones

# configure_adsb(master)
upload_mission(master, filtered_waypoints)
execute_mission(master, filtered_waypoints)

from pymavlink import mavutil
import time

def connect_vehicle(connection_string):
    """
    Connect to the vehicle.
    """
    print("Connecting to vehicle...")
    the_connection = mavutil.mavlink_connection(connection_string)
    the_connection.wait_heartbeat()
    print("Heartbeat received!")
    return the_connection

def parse_waypoint_file(file_path):
    """
    Parse a waypoint file in Mission Planner format.
    """
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Invalid waypoint file format")
    
    waypoints = []
    for line in lines[1:]:  # Skip the header
        parts = line.strip().split('\t')
        seq, current, frame, command = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
        param1, param2, param3, param4 = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
        lat, lon, alt, autocontinue = float(parts[8]), float(parts[9]), float(parts[10]), int(parts[11])
        
        waypoint = mavutil.mavlink.MAVLink_mission_item_message(
            target_system=1,
            target_component=1,
            seq=seq,
            frame=frame,
            command=command,
            current=current,
            autocontinue=autocontinue,
            param1=param1, param2=param2, param3=param3, param4=param4,
            x=lat, y=lon, z=alt
        )
        waypoints.append(waypoint)
    
    return waypoints

def upload_mission(the_connection, waypoints):
    """
    Upload waypoints to the vehicle using MISSION_ITEM_INT.
    """
    print("Uploading mission...")

    # Send the number of waypoints
    the_connection.mav.mission_count_send(
        the_connection.target_system,
        the_connection.target_component,
        len(waypoints)
    )

    # Send each waypoint using MISSION_ITEM_INT
    for waypoint in waypoints:
        the_connection.mav.mission_request_int_send(
            the_connection.target_system,
            the_connection.target_component,
            waypoint.seq
        )
        print(f"Requesting waypoint {waypoint.seq}")
        
        # Send each waypoint data after receiving MISSION_REQUEST_INT
        the_connection.mav.mission_item_int_send(
            the_connection.target_system,
            the_connection.target_component,
            waypoint.seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Use the appropriate frame
            waypoint.command,
            waypoint.current,
            waypoint.autocontinue,
            waypoint.param1, waypoint.param2, waypoint.param3, waypoint.param4,
            int(waypoint.x * 1e7),  # Latitude scaled to int32
            int(waypoint.y * 1e7),  # Longitude scaled to int32
            waypoint.z  # Altitude
        )

    print("Mission upload complete!")

def start_mission(the_connection):
    """
    Arm the vehicle and start the mission.
    """
    # Arm the drone
    the_connection.set_mode('GUIDED')
    print("Arming the vehicle...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # Confirmation
        1,  # Arm (1 to arm, 0 to disarm)
        0, 0, 0, 0, 0, 0  # Unused parameters
    )

    # Wait for the vehicle to arm
    while True:
        msg = the_connection.recv_match(type='HEARTBEAT', blocking=True)
        if msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Vehicle armed!")
            break
        print("Waiting for arming...")
        time.sleep(1)

    # Set mode to AUTO
    print("Setting mode to AUTO...")
    the_connection.set_mode('AUTO')

    # Wait until the mode is set before starting the mission
    time.sleep(2)

    # Start the mission
    print("Starting mission...")
    the_connection.mav.command_long_send(
        the_connection.target_system,
        the_connection.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,  # Confirmation
        0,  # First mission item index
        0, 0, 0, 0, 0, 0  # Unused parameters
    )

def main():
    connection_string = "172.17.160.1:14550"
    waypoint_file_path = "waypoint.waypoints"

    # Connect to the vehicle
    the_connection = connect_vehicle(connection_string)

    # Parse the waypoint file
    waypoints = parse_waypoint_file(waypoint_file_path)

    # Upload the mission
    upload_mission(the_connection, waypoints)

    # Start the mission
    start_mission(the_connection)

if __name__ == "__main__":
    main()

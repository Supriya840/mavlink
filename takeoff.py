from pymavlink import mavutil
import time

# Establish connection (replace with your connection string)
connection_string = 'udpin:172.17.160.1:14550'  # Example connection to a SITL (Software In The Loop) instance.
# For a real drone, you would use a connection like 'com14', '/dev/ttyACM0', or 'udp:127.0.0.1:14550'
master = mavutil.mavlink_connection(connection_string)

# Wait for the heartbeat message from the drone to ensure communication is established
master.wait_heartbeat()
print("Heartbeat from system (System ID: {})".format(master.target_system))

# Set the mode to GUIDED (so the drone can follow commands like takeoff)
master.set_mode('GUIDED')

# Arm the drone
def arm_drone():
    print("Arming drone...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0  # Parameters for arm command (1 means arm, 0 means disarm)
    )

# Takeoff the drone
def takeoff(altitude):
    print(f"Taking off to {altitude} meters...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude  # Takeoff command, with desired altitude
    )

# Land the drone
def land():
    print("Landing...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0  # Land command
    )

# Monitor altitude and land after reaching the target altitude
def monitor_altitude_and_land(target_altitude):
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_altitude = msg.relative_alt / 1000.0  # The altitude is in millimeters, so convert to meters.
        print(f"Current Altitude: {current_altitude} meters")

        if current_altitude >= target_altitude:
            print("Desired altitude reached, landing...")
            land()  # Command to land once the target altitude is reached
            break

        time.sleep(1)

# Arm and take off
arm_drone()
time.sleep(5)  # Wait for a few seconds to ensure the drone is armed
takeoff(10)  # Take off to 10 meters altitude
monitor_altitude_and_land(10)  # Monitor altitude and land after reaching 10 meters


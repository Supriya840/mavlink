# mavlink

SITL setup

Steps:
1. cd ardupilot
2. cd ArduCopter
3. ../Tools/autotest/sim_vehicle.py --map --console
4. ../Tools/autotest/sim_vehicle.py --map --console -l 13.0310227,77.5654036,925.640000,90 ( this is for MSRIT ) (Change home location)

Steps

-Change Mode: Set the drone mode to GUIDED.

-Arm the Drone: Arm the motors.

-Set Altitude: Command the drone to ascend to 10 meters.

-Land: Once the desired altitude is reached, initiate landing.

CIRCLE

Make the drone fly in a circular pattern.

Steps:

-Arm the Drone: Arm the motors.

-Take Off: Ascend to a safe altitude.

-Get Current Location: Retrieve the drone’s current GPS coordinates.

-Set Parameters:

-Define the radius of the circle.

-Calculate and set the waypoints for the circular path.

-Execute Command: Instruct the drone to perform 2 laps of the circle.

MISSION

Plan and execute a mission with predefined waypoints.

Steps:

-Change Waypoint File Location: Prepare the waypoint file.

-Create Waypoints:

-Use Mission Planner to design the mission path.

-Save the waypoint file.

-Upload Waypoints: Upload the mission waypoints to SITL (Software In The Loop).

-Arm the Drone: Arm the motors.

-Set Mode: Change the drone mode to AUTO.

-Start Mission: Begin the mission and monitor the drone as it follows the uploaded waypoints.




import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Function to get airspeed values
def get_airspeed_min():
    return 5  # Min speed in meters per second

def get_airspeed_max():
    return 15  # Max speed in meters per second

def get_airspeed_cruise():
    return 10  # Cruise speed in meters per second

# Function to get pitch limit value
def get_pitch_limit_max_deg():
    return 25  # Max pitch angle in degrees

# Function to get target takeoff altitude
def get_target_altitude():
    return 95  # Target takeoff altitude in meters

# Function to get connection string
def get_connection_string():
    return "127.0.0.1:14550"  # SITL connection string, change if needed

# Function to connect to the vehicle
def connect_to_vehicle(connection_string):
    print("Connecting to vehicle...")
    return connect(connection_string, wait_ready=True)

# Function to arm the vehicle and take off to the target altitude
def arm_and_takeoff(vehicle, target_altitude):
    print("Performing pre-arm checks...")
    while not vehicle.is_armable:
        print(f"Waiting for vehicle to become armable: {vehicle.is_armable}, EKF OK: {vehicle.ekf_ok}")
        time.sleep(2)

    # Set mode to GUIDED
    print("Changing to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print(f"Waiting for mode to switch to GUIDED... Current Mode: {vehicle.mode.name}")
        time.sleep(1)

    # Arm the vehicle
    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for motors to arm...")
        time.sleep(2)

    # Set airspeed before takeoff (cruise speed)
    airspeed_cruise = get_airspeed_cruise()
    print(f"Setting airspeed to {airspeed_cruise} m/s...")
    vehicle.airspeed = airspeed_cruise

    # Set pitch limit to prevent excessive tilting
    pitch_limit_max_deg = get_pitch_limit_max_deg()
    print(f"Setting pitch limit to {pitch_limit_max_deg} degrees...")
    vehicle.parameters['PITCH_MAX'] = pitch_limit_max_deg

    # Now let's try to initiate the takeoff using the MAVLink COMMAND_DO_TAKEOFF
    print("Sending MAVLink command for takeoff...")
    # Send a MAVLink takeoff command
    takeoff_command = vehicle.message_factory.command_long_encode(
        0,  # target system
        0,  # target component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # confirmation
        0, 0, 0, 0, 0, 0, target_altitude  # target altitude
    )
    vehicle.send_mavlink(takeoff_command)

    # Monitor altitude during takeoff
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {altitude:.2f} meters")
        if altitude >= target_altitude * 0.95:  # Check if within 95% of target altitude
            print("Takeoff successful!")
            break
        time.sleep(1)

# Main function to initiate takeoff
def main():
    connection_string = get_connection_string()
    vehicle = connect_to_vehicle(connection_string)
    
    target_altitude = get_target_altitude()
    arm_and_takeoff(vehicle, target_altitude)
    
    print("Test complete")

# Run the main function
if __name__ == "__main__":
    main()

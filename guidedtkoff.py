import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Constants for airspeed and pitch
AIRSPEED_MIN = 5  # Min speed in meters per second
AIRSPEED_MAX = 15  # Max speed in meters per second
AIRSPEED_CRUISE = 10  # Cruise speed in meters per second
PITCH_LIMIT_MAX_DEG = 25  # Max pitch angle in degrees

# Connection settings
connection_string = "127.0.0.1:14550"  # SITL connection string, change if needed
takeoff = 95  # Target takeoff altitude (meters)

# Connect to vehicle
print("Connecting to vehicle...")
vehicle = connect(connection_string, wait_ready=True)

# Function to arm the vehicle and take off to the target altitude
def arm_and_takeoff(target_altitude):
    print("Performing pre-arm checks...")
    while not vehicle.is_armable:
        print(f"Waiting for vehicle to become armable: {vehicle.is_armable}, EKF OK: {vehicle.ekf_ok}")
        time.sleep(2)

    # Set mode to GUIDED
    print("Changing to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("Waiting for mode to switch to GUIDED...")
        time.sleep(1)

    # Arm the vehicle
    print("Arming motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for motors to arm...")
        time.sleep(2)

    # Set airspeed before takeoff (cruise speed)
    print(f"Setting airspeed to {AIRSPEED_CRUISE} m/s...")
    vehicle.airspeed = AIRSPEED_CRUISE

    # Set pitch limit to prevent excessive tilting
    print(f"Setting pitch limit to {PITCH_LIMIT_MAX_DEG} degrees...")
    vehicle.parameters['PITCH_MAX'] = PITCH_LIMIT_MAX_DEG

    print("Motors armed. Taking off...")
    vehicle.simple_takeoff(target_altitude)

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
    arm_and_takeoff(takeoff)
    print("Test complete")

# Run the main function
if __name__ == "__main__":
    main()


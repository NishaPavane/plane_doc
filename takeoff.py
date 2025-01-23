import time
from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Connect to the SITL instance (make sure SITL is running)
connection_string = "127.0.0.1:14550"  # Change if needed
takeoff = 95 #target tkoff

# connection to vehicle
print("connecting to vehicle..")
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(target_altitude):
    print("performing pre-arm checks...")
    while not vehicle.is_armable:
        print(f"waitig for vehicle to become armable: {vehicle.is_armable}, EKF OK: {vehicle.ekf_ok}")
        time.sleep(2)
         
         
    print("CHANGING TO GUIDED MODE")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("Waiting for MODE IN GUIDED...")
        time.sleep(1)
        
    print("ARMING MOTORS...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming")
        time.sleep(2)
        
    print("motors armed. Taking off")
    vehicle.simple_takeoff(target_altitude)
     
     
    #moniter altitude
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {altitude:.2f} meters")
        if altitude >= target_altitude * 0.95:
            print("Takeoff successfull!!!")
            break
        time.sleep(1)
        
###main takeoff
 
def main():
    
    arm_and_takeoff(takeoff)
    print("Test")
    
            

    


   
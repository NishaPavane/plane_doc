from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil
import time
import math

class Plane():

    def __init__(self, connection_string=None, vehicle=None):
        connection_string = '127.0.0.1:14550'
        if not vehicle is None:
            self.vehicle = vehicle
            print("Using the provided vehicle")
        elif connection_string is not None:
            print("Connecting with vehicle...")
            self._connect(connection_string)
        else:
            raise ValueError("ERROR: a valid dronekit vehicle or a connection string must be supplied")
            return

        self._setup_listeners()

        self.airspeed = 60.0       #- [m/s] airspeed
        self.groundspeed = 0.0     #- [m/s] ground speed

        self.pos_lat = 0.0         #- [deg] latitude
        self.pos_lon = 0.0         #- [deg] longitude
        self.pos_alt_rel = 0.0     #- [m] altitude relative to takeoff
        self.pos_alt_abs = 0.0     #- [m] above mean sea level

        self.att_roll_deg = 0.0    #- [deg] roll
        self.att_pitch_deg = 0.0   #- [deg] pitch
        self.att_heading_deg = 0.0 #- [deg] magnetic heading

        self.wind_dir_to_deg = 0.0 #- [deg] wind direction (where it is going)
        self.wind_dir_from_deg = 0.0 #- [deg] wind coming from direction
        self.wind_speed = 0.0      #- [m/s] wind speed

        self.climb_rate = 0.0      #- [m/s] climb rate
        self.throttle = 0.0        #- [] throttle (0-100)

        self.ap_mode = ''          #- [] Autopilot flight mode

        self.mission = self.vehicle.commands #-- mission items

        self.location_home = LocationGlobalRelative(0, 0, 0)  #- LocationRelative type home
        self.location_current = LocationGlobalRelative(0, 0, 0)  #- LocationRelative type current position

    def _connect(self, connection_string):
        self.vehicle = connect(connection_string, wait_ready=True, heartbeat_timeout=60)
        self._setup_listeners()

    def _setup_listeners(self):
        #----------------------------
        #--- CALLBACKS
        #----------------------------
        @self.vehicle.on_message('ATTITUDE')
        def listener(vehicle, name, message):  #--- Attitude
            self.att_roll_deg = math.degrees(message.roll)
            self.att_pitch_deg = math.degrees(message.pitch)
            self.att_heading_deg = math.degrees(message.yaw) % 360

        @self.vehicle.on_message('GLOBAL_POSITION_INT')
        def listener(vehicle, name, message):  #--- Position / Velocity
            self.pos_lat = message.lat * 1e-7
            self.pos_lon = message.lon * 1e-7
            self.pos_alt_rel = message.relative_alt * 1e-3
            self.pos_alt_abs = message.alt * 1e-3
            self.location_current = LocationGlobalRelative(self.pos_lat, self.pos_lon, self.pos_alt_rel)

        @self.vehicle.on_message('VFR_HUD')
        def listener(vehicle, name, message):  #--- HUD
            self.airspeed = message.airspeed
            self.groundspeed = message.groundspeed
            self.throttle = message.throttle
            self.climb_rate = message.climb

        @self.vehicle.on_message('WIND')
        def listener(vehicle, name, message):  #--- WIND
            self.wind_speed = message.speed
            self.wind_dir_from_deg = message.direction % 360
            self.wind_dir_to_deg = (self.wind_dir_from_deg + 180) % 360

        print(">> Connection Established")

    def is_armed(self):  #-- Check whether uav is armed
        """ Checks whether the UAV is armed """
        return self.vehicle.armed

    def arm(self):  #-- arm the UAV
        """ Arm the UAV """
        self.vehicle.armed = True

    def disarm(self):  #-- disarm UAV
        """ Disarm the UAV """
        self.vehicle.armed = False

    def set_ap_mode(self, mode):  #--- Set Autopilot mode
        """ Set Autopilot mode """
        tgt_mode = VehicleMode(mode)
        self.vehicle.mode = tgt_mode

    def clear_mission(self):  #--- Clear the onboard mission
        """ Clear the current mission. """
        self.vehicle.commands.clear()
        self.vehicle.flush()
        self.mission = self.vehicle.commands
        self.mission.download()
        self.mission.wait_ready()

    def mission_add_takeoff(self, takeoff_altitude=50, takeoff_pitch=15, heading=None):
        """ Adds a takeoff item to the UAV mission """
        if heading is None:
            heading = self.att_heading_deg

        self.clear_mission()
        takeoff_item = Command(0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, takeoff_pitch, 0, 0, heading, 0, 0, takeoff_altitude)
        self.mission.add(takeoff_item)
        self.vehicle.flush()

    def arm_and_takeoff(self, altitude=50, pitch_deg=12):
        self.mission_add_takeoff(takeoff_altitude=1.5 * altitude, takeoff_pitch=pitch_deg)

        while not self.vehicle.is_armable:
            print("Wait to be armable...")
            time.sleep(1.0)

        while self.pos_lat == 0.0:
            time.sleep(0.5)
            print("Waiting for good GPS...")

        self.location_home = LocationGlobalRelative(self.pos_lat, self.pos_lon, altitude)
        self.set_ap_mode("MANUAL")

        while not self.vehicle.armed:
            print("Try to arm...")
            self.arm()
            time.sleep(2.0)

        if self.vehicle.armed:
            print("ARMED")
            self.set_ap_mode("AUTO")
            while self.pos_alt_rel <= altitude - 20.0:
                print("Altitude = %.0f" % self.pos_alt_rel)
                time.sleep(2.0)

            self.set_ap_mode("GUIDED")
            time.sleep(1.0)
            self.vehicle.simple_goto(self.location_home)

        return True

    def set_waypoints_and_start_mission(self):
        waypoints = [
            (37.7749, -122.4194, 100),  # Example waypoint 1
            (37.7750, -122.4195, 100),  # Example waypoint 2
            (37.7751, -122.4196, 100),  # Example waypoint 3
        ]

        self.clear_mission()
        for wp in waypoints:
            lat, lon, alt = wp
            # Convert latitude, longitude, and altitude to integer values
            waypoint = Command(0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, 37.7749, -122.4194, 100)
            self.mission.add(waypoint)

        self.vehicle.flush()

        print("Mission set, now starting it.")
        self.set_ap_mode("AUTO")

    def monitor_altitude(self):
        while True:
            print(f"Current altitude: {self.pos_alt_rel} meters")
            time.sleep(2)

if __name__ == '__main__':
    plane = Plane()

    # -- Arm and takeoff
    if not plane.is_armed():
        plane.arm_and_takeoff()

    time.sleep(5)

    # -- Set waypoints and start the mission
    plane.set_waypoints_and_start_mission()

    # -- Monitor altitude during the mission
    plane.monitor_altitude()
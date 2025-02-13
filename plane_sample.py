from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, Battery, LocationGlobal, Attitude
from pymavlink import mavutil

import time
import math
import numpy as np
import psutil
import argparse
import copy

class Plane():

    def __init__(self, connection_string=None, vehicle=None):
        connection_string = '127.0.0.1:14550'
        #---- Connecting with the vehicle, using either the provided vehicle or the connection string
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
        if True:
            #---- DEFINE CALLBACKS HERE!!!
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

    def _get_location_metres(self, original_location, dNorth, dEast, is_global=False):
        earth_radius = 6378137.0  # Radius of "spherical" earth
        # Coordinate offsets in radians
        dLat = dNorth / earth_radius
        dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))

        # New position in decimal degrees
        newlat = original_location.lat + (dLat * 180 / math.pi)
        newlon = original_location.lon + (dLon * 180 / math.pi)

        if is_global:
            return LocationGlobal(newlat, newlon, original_location.alt)
        else:
            return LocationGlobalRelative(newlat, newlon, original_location.alt)

    def is_armed(self):  #-- Check whether uav is armed
        """ Checks whether the UAV is armed """
        return self.vehicle.armed

    def arm(self):  #-- arm the UAV
        """ Arm the UAV """
        self.vehicle.armed = True

    def disarm(self):  #-- disarm UAV
        """ Disarm the UAV """
        self.vehicle.armed = False

    def set_airspeed(self, speed):  #--- Set target airspeed
        """ Set uav airspeed m/s """
        self.vehicle.airspeed = speed

    def set_ap_mode(self, mode):  #--- Set Autopilot mode
        """ Set Autopilot mode """
        time_0 = time.time()
        try:
            tgt_mode = VehicleMode(mode)
        except:
            return False

        while self.get_ap_mode() != tgt_mode:
            self.vehicle.mode = tgt_mode
            time.sleep(0.2)
            if time.time() > time_0 + 5:
                return False

        return True

    def get_ap_mode(self):  #--- Get the autopilot mode
        """ Get the autopilot mode """
        self._ap_mode = self.vehicle.mode
        return self.vehicle.mode

    def clear_mission(self):  #--- Clear the onboard mission
        """ Clear the current mission. """
        cmds = self.vehicle.commands
        self.vehicle.commands.clear()
        self.vehicle.flush()
        self.mission = self.vehicle.commands
        self.mission.download()
        self.mission.wait_ready()

    def download_mission(self):  #--- download the mission
        """ Download the current mission from the vehicle. """
        self.vehicle.commands.download()
        self.vehicle.commands.wait_ready()  # wait until download is complete.
        self.mission = self.vehicle.commands

    def mission_add_takeoff(self, takeoff_altitude=50, takeoff_pitch=15, heading=None):
        """ Adds a takeoff item to the UAV mission, if it's not defined yet """
        if heading is None:
            heading = self.att_heading_deg

        self.download_mission()
        #-- save the mission: copy in the memory
        tmp_mission = list(self.mission)

        is_mission = False
        if len(tmp_mission) >= 1:
            is_mission = True
            print("Current mission:")
            for item in tmp_mission:
                print(item)
            #-- If takeoff already in the mission, do not do anything

        if is_mission and tmp_mission[0].command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
            print("Takeoff already in the mission")
        else:
            print("Takeoff not in the mission: adding")
            self.clear_mission()
            takeoff_item = Command(0, 0, 0, 3, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, takeoff_pitch, 0, 0, heading, 0, 0, takeoff_altitude)
            self.mission.add(takeoff_item)
            for item in tmp_mission:
                self.mission.add(item)
            self.vehicle.flush()
            print(">>>>>Done")

    def arm_and_takeoff(self, altitude=50, pitch_deg=12):
        self.mission_add_takeoff(takeoff_altitude=1.5 * altitude, takeoff_pitch=pitch_deg)
        print("Takeoff mission ready")

        while not self.vehicle.is_armable:
            print("Wait to be armable...")
            time.sleep(1.0)

        #-- Save home
        while self.pos_lat == 0.0:
            time.sleep(0.5)
            print("Waiting for good GPS...")
        self.location_home = LocationGlobalRelative(self.pos_lat, self.pos_lon, altitude)

        print("Home is saved as "), self.location_home
        print("Vehicle is Armable: try to arm")
        self.set_ap_mode("MANUAL")
        n_tries = 0
        while not self.vehicle.armed:
            print("Try to arm...")
            self.arm()
            n_tries += 1
            time.sleep(2.0)

            if n_tries > 5:
                print("!!! CANNOT ARM")
                break

        #--- Set to auto and check the ALTITUDE
        if self.vehicle.armed:
            print("ARMED")
            self.set_ap_mode("AUTO")

            while self.pos_alt_rel <= altitude - 20.0:
                print("Altitude = %.0f" % self.pos_alt_rel)
                time.sleep(2.0)

            print("Altitude reached: set to GUIDED")
            self.set_ap_mode("GUIDED")

            time.sleep(1.0)

            print("Sending to the home")
            self.vehicle.simple_goto(self.location_home)

        return True

    def get_target_from_bearing(self, original_location, ang, dist, altitude=None):
        if altitude is None:
            altitude = original_location.alt

        # print '---------------------- simulate_target_packet'
        dNorth = dist * math.cos(ang)
        dEast = dist * math.sin(ang)
        # print "Based on the actual heading of %.0f, the relative target's coordinates are %.1f m North, %.1f m East" % (math.degrees(ang), dNorth, dEast)

        #-- Get the Lat and Lon
        tgt = self._get_location_metres(original_location, dNorth, dEast)

        tgt.alt = altitude
        # print "Obtained the following target", tgt.lat, tgt.lon, tgt.alt

        return tgt

    def ground_course_2_location(self, angle_deg, altitude=None):
        tgt = self.get_target_from_bearing(original_location=self.location_current,
                                           ang=math.radians(angle_deg),
                                           dist=5000,
                                           altitude=altitude)
        return tgt

    def goto(self, location):
        self.vehicle.simple_goto(location)

    def set_ground_course(self, angle_deg, altitude=None):
        self.goto(self.ground_course_2_location(angle_deg, altitude))

    def set_waypoints_and_start_mission(self):
        waypoints = [
            (37.7749, -122.4194, 100),  # Example waypoint 1
            (37.7750, -122.4195, 100),  # Example waypoint 2
            (37.7751, -122.4196, 100),  # Example waypoint 3
        ]

        # Set each waypoint and go to it
        for wp in waypoints:
            lat, lon, alt = wp
            target_location = LocationGlobalRelative(lat, lon, alt)
            print(f"Going to waypoint: {lat}, {lon}, {alt}")
            self.goto(target_location)  # Send the drone to the waypoint
            time.sleep(20)  # Wait for the drone to reach the waypoint


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='tcp:127.0.0.1:5762')
    args = parser.parse_args()

    connection_string = args.connect

    #-- Create the object
    plane = Plane(connection_string)

    #-- Arm and takeoff
    if not plane.is_armed():
        plane.arm_and_takeoff()

    time.sleep(5)

    #-- Set in fbwb and test the rc override
    plane.set_ap_mode("FBWB")
    time.sleep(3.0)

    #-- Set in Guided and test the ground course
    plane.set_ap_mode("GUIDED")

    cruise_altitude = 100
    cruise_angle_deg = 0.0
    delta_angle_deg = 40.0

    while True:
        print("Heading to %.0fdeg" % cruise_angle_deg)
        plane.set_ground_course(cruise_angle_deg, cruise_altitude)
        time.sleep(5)
        cruise_angle_deg += delta_angle_deg

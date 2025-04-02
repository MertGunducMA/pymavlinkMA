#!/usr/bin/env python3
"""
Oil Rig Helipad Guidance and Landing System
This script implements guidance and landing control for an autonomous helicopter
approaching and landing on an oil rig helipad using MAVLink communication.
"""

import time
from pymavlink import mavutil

class OilRigGuidance:
    def __init__(self, connection_string='udpin:localhost:14551'):
        """Initialize the guidance system with MAVLink connection."""
        print(f"Connecting to vehicle on {connection_string}")
        self.master = mavutil.mavlink_connection(connection_string)
        
        # Wait for the first heartbeat
        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print("Heartbeat from system (system %u component %u)" % 
              (self.master.target_system, self.master.target_component))
        
        # Get autopilot type
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True)
        self.autopilot_type = msg.autopilot
        print(f"Detected autopilot type: {self.autopilot_type}")
        
        # Wait for GPS lock
        self.wait_for_position_aiding()
        
    def wait_for_position_aiding(self):
        """Wait for GPS lock."""
        print("Waiting for position aiding...")
        while True:
            msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True)
            if msg.fix_type >= 3:  # 3D fix or better
                print("Position aiding active")
                break
            time.sleep(0.1)
        
    def set_mode(self, mode):
        """Set the flight mode."""
        print(f"Setting mode to {mode}...")
        
        # Get mode ID from mode mapping
        try:
            mode_id = self.master.mode_mapping()[mode]
            print(f"Mode ID for {mode}: {mode_id}")
        except:
            print(f"Failed to get mode ID for {mode}")
            return False
            
        # Send mode change command
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,  # confirmation
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
            0,  # custom sub-mode
            0,  # unused
            0,  # unused
            0,  # unused
            0   # unused
        )
        
        # Wait for ACK
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg:
            success = ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            print(f"Mode {mode} set: {'Success' if success else 'Failed'}")
            return success
        else:
            print(f"No ACK received for mode {mode} set")
            return False
        
    def arm(self):
        """Arm the vehicle."""
        print("Attempting to arm vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0,  # force
            0,  # unused
            0,  # unused
            0,  # unused
            0,  # unused
            0   # unused
        )
        
        # Wait for ACK
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg:
            success = ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            print(f"Arm command: {'Success' if success else 'Failed'}")
            return success
        else:
            print("No ACK received for arm command")
            return False

    def disarm(self):
        """Disarm the vehicle."""
        print("Attempting to disarm vehicle...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            0,  # force
            0,  # unused
            0,  # unused
            0,  # unused
            0,  # unused
            0   # unused
        )
        
        # Wait for ACK
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg:
            success = ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            print(f"Disarm command: {'Success' if success else 'Failed'}")
            return success
        else:
            print("No ACK received for disarm command")
            return False

    def takeoff(self, altitude_m=20):
        """Takeoff to specified altitude in meters."""
        print(f"Taking off to {altitude_m} meters...")
        
        # Set appropriate mode for takeoff
        if not self.set_mode('GUIDED'):
            print("Failed to enter GUIDED mode")
            return False
            
        # Arm the vehicle if not armed
        if not self.arm():
            print("Failed to arm vehicle")
            return False
            
        # Send takeoff command with simple parameters
        print("Sending takeoff command...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # param1 (pitch)
            0,  # param2
            0,  # param3
            0,  # param4 (yaw)
            0,  # param5 (latitude)
            0,  # param6 (longitude)
            altitude_m  # param7 (altitude)
        )
        
        # Wait for ACK
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if not ack_msg:
            print("No ACK received for takeoff command")
            return False
            
        success = ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
        print(f"Takeoff command: {'Success' if success else 'Failed'}")
        
        if not success:
            return False
            
        # Wait until we reach the target altitude
        print("Monitoring altitude...")
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            current_alt = msg.relative_alt / 1000.0  # Convert mm to meters
            # print(f"Current altitude: {current_alt:.2f}m")
            if current_alt >= altitude_m * 0.95:  # Within 5% of target altitude
                print("Reached target altitude")
                return True
            time.sleep(0.1)

    def land(self):
        """Execute landing sequence."""
        print("Initiating landing sequence...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # confirmation
            0,  # abort altitude
            0,  # precision landing
            0,  # unused
            0,  # yaw angle
            0,  # latitude
            0,  # longitude
            0   # altitude
        )
        
        # Wait for ACK
        ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        if ack_msg:
            success = ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            print(f"Land command: {'Success' if success else 'Failed'}")
        else:
            print("No ACK received for land command")
            return False
            
        # Monitor landing
        print("Monitoring landing...")
        while True:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            current_alt = msg.relative_alt / 1000.0  # Convert mm to meters
            # print(f"Current altitude: {current_alt:.2f}m")
            if current_alt < 0.1:  # Less than 10cm from ground
                print("Landed successfully")
                break
            time.sleep(0.1)

    def get_current_position(self):
        """Get current position of the vehicle."""
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        return {
            'lat': msg.lat / 1e7,  # Convert from degE7 to degrees
            'lon': msg.lon / 1e7,
            'alt': msg.alt / 1000.0,  # Convert from mm to meters
            'relative_alt': msg.relative_alt / 1000.0
        }

    def get_distance_bearing(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance and bearing between two GPS coordinates.
        Returns: (distance in meters, bearing in degrees)
        """
        from math import sin, cos, sqrt, atan2, radians

        R = 6371000  # Earth's radius in meters

        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        # Distance calculation (Haversine formula)
        a = sin(dlat/2)**2 + cos(lat1) * cos(lat2) * sin(dlon/2)**2
        c = 2 * atan2(sqrt(a), sqrt(1-a))
        distance = R * c
        
        # Bearing calculation
        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        bearing = atan2(y, x)
        bearing_deg = (bearing * 180/3.14159 + 360) % 360
        
        return distance, bearing_deg

    def set_guided_waypoint(self, lat, lon, alt):
        """Send vehicle to specified waypoint using position and velocity control."""
        print(f"Setting target position: lat={lat}, lon={lon}, alt={alt}")
        
        # Convert lat/lon to int for mavlink message (degrees * 1e7)
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        alt_int = int(alt * 1000)  # Convert to millimeters

        # First, set GUIDED mode again to ensure we're in the right mode
        if not self.set_mode('GUIDED'):
            print("Failed to set GUIDED mode for waypoint")
            return False

        # Calculate bearing and distance to target
        current_pos = self.get_current_position()
        distance, bearing = self.get_distance_bearing(
            current_pos['lat'],
            current_pos['lon'],
            lat,
            lon
        )
        print(f"Distance to target: {distance:.1f}m, Bearing: {bearing:.1f}°")

        # Set target position with velocity control
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        # Send command with both position and velocity
        print("Sending position target command...")
        self.master.mav.set_position_target_global_int_send(
            0,                      # timestamp (0 for immediate)
            self.master.target_system,  # target system
            self.master.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            type_mask,              # type mask (use pos and vel)
            lat_int,               # latitude (degrees * 1e7)
            lon_int,               # longitude (degrees * 1e7)
            alt_int/1000.0,        # altitude (meters, converted from mm)
            2.0,                   # vx (m/s)
            2.0,                   # vy (m/s)
            0.0,                   # vz (m/s)
            0, 0, 0,               # acceleration (ignored)
            bearing * 0.0174533,   # yaw (radians)
            0                      # yaw_rate (ignored)
        )

        # Wait a moment for the command to be processed
        time.sleep(2)

        # Verify the vehicle is moving to the target
        start_time = time.time()
        timeout = 20  # seconds to wait for movement to begin
        movement_detected = False
        last_distance = distance
        
        print("\nMonitoring movement...")
        while time.time() - start_time < timeout:
            current_pos = self.get_current_position()
            current_distance, current_bearing = self.get_distance_bearing(
                current_pos['lat'],
                current_pos['lon'],
                lat,
                lon
            )
            
            # Calculate movement
            distance_change = last_distance - current_distance
            last_distance = current_distance
            
            print(f"\rDistance: {current_distance:.1f}m, Change: {distance_change:.1f}m, Bearing: {current_bearing:.1f}°", end='')
            
            # Check if we're moving towards the target
            if distance_change > 0.5:  # Moving at least 0.5 meters closer
                movement_detected = True
                print("\nVehicle is moving towards target")
                break
                
            # If we're very close to target, consider it reached
            if current_distance < 5:  # Within 5 meters
                print("\nReached target position")
                return True
                
            # Send the command again every 5 seconds if no movement
            if (time.time() - start_time) % 5 < 0.1:
                print("\nResending position command...")
                self.master.mav.set_position_target_global_int_send(
                    0, self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask, lat_int, lon_int, alt_int/1000.0,
                    2.0, 2.0, 0.0, 0, 0, 0,
                    bearing * 0.0174533, 0
                )
            
            time.sleep(1)
        
        if not movement_detected:
            print("\nVehicle not responding to position command")
            return False
            
        print("\nWaypoint command accepted, continuing to target")
        return True

    def approach_helipad(self, helipad_lat, helipad_lon, approach_alt=100, offset_distance=200):
        """
        Approach the helipad GPS location and stop at specified offset.
        
        Args:
            helipad_lat (float): Helipad latitude in degrees
            helipad_lon (float): Helipad longitude in degrees
            approach_alt (float): Approach altitude in meters AGL
            offset_distance (float): Distance to stop from helipad in meters
        """
        print(f"Starting approach to helipad at {helipad_lat}, {helipad_lon}")
        
        # Ensure we're in GUIDED mode
        if not self.set_mode('GUIDED'):
            print("Failed to enter GUIDED mode")
            return False
            
        # Get current position
        current_pos = self.get_current_position()
        print(f"Current position: {current_pos['lat']}, {current_pos['lon']}, alt: {current_pos['relative_alt']}m")
        
        # Calculate distance and bearing to helipad
        distance, bearing = self.get_distance_bearing(
            current_pos['lat'], 
            current_pos['lon'], 
            helipad_lat, 
            helipad_lon
        )
        print(f"Distance to helipad: {distance:.1f}m, Bearing: {bearing:.1f}°")
        
        # If we're closer than the offset distance, move away to safe distance
        if distance < offset_distance:
            print("Too close to helipad, moving to safe distance")
            # TODO: Implement backoff logic
            return False
            
        # Calculate approach waypoint (offset_distance meters from helipad)
        from math import sin, cos, radians, degrees, asin, atan2
        
        # Convert offset distance from meters to degrees
        R = 6371000  # Earth's radius in meters
        angular_distance = offset_distance / R
        
        helipad_lat_rad = radians(helipad_lat)
        helipad_lon_rad = radians(helipad_lon)
        bearing_rad = radians(bearing)
        
        # Calculate approach point coordinates
        # This is the point offset_distance meters away from helipad in the opposite direction
        approach_lat_rad = asin(
            sin(helipad_lat_rad) * cos(angular_distance) +
            cos(helipad_lat_rad) * sin(angular_distance) * cos(bearing_rad + 3.14159)
        )
        
        approach_lon_rad = helipad_lon_rad + atan2(
            sin(bearing_rad + 3.14159) * sin(angular_distance) * cos(helipad_lat_rad),
            cos(angular_distance) - sin(helipad_lat_rad) * sin(approach_lat_rad)
        )
        
        approach_lat = degrees(approach_lat_rad)
        approach_lon = degrees(approach_lon_rad)
        
        print(f"Calculated approach point: {approach_lat}, {approach_lon}, alt: {approach_alt}m")
        
        # Command vehicle to approach point
        if not self.set_guided_waypoint(approach_lat, approach_lon, approach_alt):
            print("Failed to set approach waypoint")
            return False
            
        # Monitor progress
        print("Monitoring approach...")
        while True:
            current_pos = self.get_current_position()
            distance_to_target, _ = self.get_distance_bearing(
                current_pos['lat'],
                current_pos['lon'],
                approach_lat,
                approach_lon
            )
            
            print(f"Distance to approach point: {distance_to_target:.1f}m")
            
            # Check if we've reached the approach point (within 5 meters)
            if distance_to_target < 5:
                print("Reached approach point")
                return True
                
            time.sleep(1)  # Update every second
            
        return True

    # TODO: Add more methods for oil rig specific operations
    # def approach_helipad(self):
    #     """Navigate to helipad approach point."""
    #     pass
    
    # def align_with_helipad(self):
    #     """Align vehicle with helipad orientation."""
    #     pass
    
    # def precision_landing(self):
    #     """Execute precision landing on helipad."""
    #     pass 
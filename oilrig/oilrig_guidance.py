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
        """Send vehicle to specified waypoint using position control."""
        print("\n=== Starting Waypoint Navigation ===")
        print(f"Target position: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
        
        # Convert lat/lon to int for mavlink message (degrees * 1e7)
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        alt_int = int(alt * 1000)  # Convert to millimeters

        # First, set GUIDED mode again to ensure we're in the right mode
        if not self.set_mode('GUIDED'):
            print("Failed to set GUIDED mode for waypoint")
            return False

        # Calculate bearing to target
        current_pos = self.get_current_position()
        distance, bearing = self.get_distance_bearing(
            current_pos['lat'],
            current_pos['lon'],
            lat,
            lon
        )
        print("\nInitial Position:")
        print(f"Current: lat={current_pos['lat']:.6f}, lon={current_pos['lon']:.6f}, alt={current_pos['relative_alt']:.1f}m")
        print(f"Distance to target: {distance:.1f}m, Bearing: {bearing:.1f}°")

        # Set type mask for position control only
        # Ignore velocity, acceleration, and yaw rate
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )

        # Send command with position only
        print("\nSending position target command...")
        self.master.mav.set_position_target_global_int_send(
            0,                      # timestamp (0 for immediate)
            self.master.target_system,  # target system
            self.master.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            type_mask,              # type mask (use pos only)
            lat_int,               # latitude (degrees * 1e7)
            lon_int,               # longitude (degrees * 1e7)
            alt,                   # altitude (meters)
            0.0,                   # vx (ignored)
            0.0,                   # vy (ignored)
            0.0,                   # vz (ignored)
            0, 0, 0,               # acceleration (ignored)
            bearing * 0.0174533,   # yaw (radians)
            0                      # yaw_rate (ignored)
        )

        # Monitor progress
        last_distance = distance
        last_print_time = time.time()
        command_send_time = time.time()
        
        print("\nMonitoring movement to target...")
        try:
            while True:
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
                
                # Update progress every second
                current_time = time.time()
                if current_time - last_print_time >= 1.0:
                    print("\nPosition Update:")
                    print(f"Current: lat={current_pos['lat']:.6f}, lon={current_pos['lon']:.6f}, alt={current_pos['relative_alt']:.1f}m")
                    print(f"Target:  lat={lat:.6f}, lon={lon:.6f}, alt={alt:.1f}m")
                    print(f"Distance: {current_distance:.1f}m, Change: {distance_change:.1f}m/s")
                    last_print_time = current_time
                
                # Check if we've reached the target (within 2 meters)
                if current_distance < 2:
                    print("\n=== Target Position Reached ===")
                    print(f"Final position: lat={current_pos['lat']:.6f}, lon={current_pos['lon']:.6f}, alt={current_pos['relative_alt']:.1f}m")
                    print("Waiting for approach alignment...")
                    return True
                    
                # Resend command every 3 seconds to ensure it's maintained
                if current_time - command_send_time >= 3.0:
                    self.master.mav.set_position_target_global_int_send(
                        0, self.master.target_system, self.master.target_component,
                        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                        type_mask, lat_int, lon_int, alt,
                        0.0, 0.0, 0.0, 0, 0, 0,
                        bearing * 0.0174533, 0
                    )
                    command_send_time = current_time
                
                time.sleep(0.1)  # Small sleep to prevent CPU overload
                
        except KeyboardInterrupt:
            print("\n=== Operation Cancelled by User ===")
            return False

    def approach_helipad(self, helipad_lat, helipad_lon, approach_alt=100, offset_distance=200):
        """
        Approach the helipad GPS location and stop at specified offset.
        """
        print("\n=== Starting Helipad Approach ===")
        print(f"Helipad position: lat={helipad_lat:.6f}, lon={helipad_lon:.6f}")
        print(f"Approach altitude: {approach_alt}m")
        print(f"Offset distance: {offset_distance}m")
        
        # Ensure we're in GUIDED mode
        if not self.set_mode('GUIDED'):
            print("Failed to enter GUIDED mode")
            return False
            
        # Get current position
        current_pos = self.get_current_position()
        print("\nInitial Position:")
        print(f"Current: lat={current_pos['lat']:.6f}, lon={current_pos['lon']:.6f}, alt={current_pos['relative_alt']:.1f}m")
        
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
            print("\nWarning: Too close to helipad, need to move to safe distance")
            # TODO: Implement backoff logic if needed
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
        
        print("\n=== Approach Point Calculated ===")
        print(f"Approach position: lat={approach_lat:.6f}, lon={approach_lon:.6f}, alt={approach_alt}m")
        
        # Command vehicle to approach point
        if not self.set_guided_waypoint(approach_lat, approach_lon, approach_alt):
            print("Failed to reach approach point")
            return False
            
        print("\n=== Approach Complete ===")
        print("Vehicle in position for final approach")
        print("Waiting for alignment confirmation...")
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
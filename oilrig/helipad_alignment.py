#!/usr/bin/env python3
"""
Helipad Alignment Guidance System
This module handles the alignment phase of the approach, creating an arc-based path
to align the vehicle with the helipad's heading while maintaining constant radius.
"""

import math
import time
from pymavlink import mavutil

class HelipadAlignment:
    def __init__(self, connection_string='udpin:localhost:14551', existing_connection=None):
        """
        Initialize the alignment system with MAVLink connection.
        
        Args:
            connection_string (str): MAVLink connection string (used if existing_connection is None)
            existing_connection: Existing MAVLink connection to use (optional)
        """
        if existing_connection:
            print("Using existing MAVLink connection")
            self.master = existing_connection
        else:
            print(f"Connecting to vehicle on {connection_string}")
            self.master = mavutil.mavlink_connection(connection_string)
            self.master.wait_heartbeat()
            print("Heartbeat received")
        
    def calculate_alignment_vectors(self, helipad_lat, helipad_lon, helipad_heading, current_lat, current_lon, radius=200):
        """
        Calculate vectors for alignment:
        1. Target vector: From helipad center along helipad heading with radius magnitude
        2. Current vector: From helipad center to current aircraft position
        
        Returns:
            tuple: (target_pos, angle_diff, clockwise)
            - target_pos: (lat, lon) of final alignment position
            - angle_diff: angle between current and target vectors in degrees
            - clockwise: True if clockwise rotation is shorter
        """
        # Convert helipad heading to radians
        heading_rad = math.radians(helipad_heading)
        
        # Calculate target position (final alignment position)
        # Using helipad heading to determine the point 200m away along that vector
        R = 6371000  # Earth radius in meters
        angular_distance = radius / R
        
        target_lat = math.asin(
            math.sin(math.radians(helipad_lat)) * math.cos(angular_distance) +
            math.cos(math.radians(helipad_lat)) * math.sin(angular_distance) * math.cos(heading_rad)
        )
        target_lon = math.radians(helipad_lon) + math.atan2(
            math.sin(heading_rad) * math.sin(angular_distance) * math.cos(math.radians(helipad_lat)),
            math.cos(angular_distance) - math.sin(math.radians(helipad_lat)) * math.sin(target_lat)
        )
        
        target_lat = math.degrees(target_lat)
        target_lon = math.degrees(target_lon)
        
        # Calculate current vector angle (from helipad to aircraft)
        current_bearing = self.get_bearing(helipad_lat, helipad_lon, current_lat, current_lon)
        current_bearing_rad = math.radians(current_bearing)
        
        # Calculate angle difference between vectors
        angle_diff = (math.degrees(heading_rad - current_bearing_rad) + 180) % 360 - 180
        
        # Determine rotation direction (clockwise if angle difference is negative)
        clockwise = angle_diff < 0
        
        return (target_lat, target_lon), abs(angle_diff), clockwise

    def calculate_arc_waypoints(self, helipad_lat, helipad_lon, helipad_heading, 
                              current_lat, current_lon, radius=200, waypoint_spacing=30):
        """
        Calculate waypoints along arc from current position to alignment position.
        Heading will naturally align by maintaining ROI towards helipad center.
        """
        # Calculate target position and rotation parameters
        (target_lat, target_lon), angle_diff, clockwise = self.calculate_alignment_vectors(
            helipad_lat, helipad_lon, helipad_heading, current_lat, current_lon, radius
        )
        
        # Calculate number of waypoints based on arc length and increased spacing
        arc_length = (angle_diff * math.pi / 180) * radius
        num_points = max(int(arc_length / waypoint_spacing), 2)  # Minimum 2 points
        angle_step = math.radians(angle_diff) / num_points
        
        waypoints = []
        R = 6371000  # Earth radius in meters
        
        # Start from current position
        current_bearing = self.get_bearing(helipad_lat, helipad_lon, current_lat, current_lon)
        current_bearing_rad = math.radians(current_bearing)
        
        # Generate waypoints along the arc
        for i in range(num_points + 1):
            if clockwise:
                angle = current_bearing_rad - (i * angle_step)
            else:
                angle = current_bearing_rad + (i * angle_step)
            
            # Calculate waypoint position
            angular_distance = radius / R
            wp_lat = math.asin(
                math.sin(math.radians(helipad_lat)) * math.cos(angular_distance) +
                math.cos(math.radians(helipad_lat)) * math.sin(angular_distance) * math.cos(angle)
            )
            wp_lon = math.radians(helipad_lon) + math.atan2(
                math.sin(angle) * math.sin(angular_distance) * math.cos(math.radians(helipad_lat)),
                math.cos(angular_distance) - math.sin(math.radians(helipad_lat)) * math.sin(wp_lat)
            )
            
            wp_lat = math.degrees(wp_lat)
            wp_lon = math.degrees(wp_lon)
            
            waypoints.append((wp_lat, wp_lon))
        
        return waypoints

    def execute_alignment(self, helipad_lat, helipad_lon, helipad_heading, altitude):
        """Execute the alignment maneuver using arc waypoints."""
        start_time = time.time()
        
        print("\n=== Starting Helipad Alignment ===")
        print(f"Helipad position: lat={helipad_lat:.6f}, lon={helipad_lon:.6f}")
        print(f"Helipad heading: {helipad_heading:.1f}°")
        
        # Get current position
        current_pos = self.get_current_position()
        if not current_pos:
            print("Failed to get current position")
            return False, 0
        
        # Calculate vectors and alignment parameters
        (target_lat, target_lon), angle_diff, clockwise = self.calculate_alignment_vectors(
            helipad_lat, helipad_lon, helipad_heading,
            current_pos['lat'], current_pos['lon']
        )
        
        print("\n=== Alignment Parameters ===")
        print(f"Current position: lat={current_pos['lat']:.6f}, lon={current_pos['lon']:.6f}")
        print(f"Target position: lat={target_lat:.6f}, lon={target_lon:.6f}")
        print(f"Angle to cover: {angle_diff:.1f}°")
        print(f"Rotation direction: {'Clockwise' if clockwise else 'Counter-clockwise'}")
        
        # Calculate waypoints
        waypoints = self.calculate_arc_waypoints(
            helipad_lat, helipad_lon, helipad_heading,
            current_pos['lat'], current_pos['lon']
        )
        
        print(f"\nGenerated {len(waypoints)} waypoints for alignment arc")
        
        # Execute waypoints
        for i, (wp_lat, wp_lon) in enumerate(waypoints):
            print(f"\n--- Waypoint {i+1}/{len(waypoints)} ---")
            print(f"Position: lat={wp_lat:.6f}, lon={wp_lon:.6f}")
            
            if not self.goto_waypoint(wp_lat, wp_lon, altitude, helipad_lat, helipad_lon):
                elapsed_time = time.time() - start_time
                print("Failed to reach waypoint")
                return False, elapsed_time
            
            time.sleep(1)
        
        # At final position, wait 5 seconds
        print("\nReached final position. Stabilizing for 5 seconds...")
        time.sleep(5)
        
        # Now set the final heading (opposite of helipad heading)
        final_heading = (helipad_heading + 180) % 360
        print(f"\nSetting final heading to {final_heading}° (opposite of helipad heading)")
        
        # Set final heading without moving position
        if not self.set_final_heading(final_heading):
            print("Failed to set final heading")
            return False, time.time() - start_time
        
        elapsed_time = time.time() - start_time
        
        # Verify final position and heading
        final_pos = self.get_current_position()
        if final_pos:
            print("\n=== Final Position Verification ===")
            print(f"Position: lat={final_pos['lat']:.6f}, lon={final_pos['lon']:.6f}")
            print(f"Final heading: {final_heading}°")
            print(f"Alignment duration: {elapsed_time:.1f} seconds")
        
        print("\n=== Alignment Complete ===")
        print("Vehicle aligned with helipad heading")
        print("Waiting for precision landing command...")
        return True, elapsed_time

    def set_guided_mode(self):
        """Ensure vehicle is in GUIDED mode."""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            4,  # GUIDED mode
            0, 0, 0, 0, 0
        )
        # Wait for command acknowledgment
        for _ in range(3):  # Try up to 3 times
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                return True
        return False

    def goto_waypoint(self, lat, lon, alt, helipad_lat, helipad_lon):
        """
        Go to specified waypoint while maintaining ROI on helipad.
        No explicit heading commands - let ROI control the heading.
        """
        # Convert coordinates for mavlink
        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)
        
        # Set ROI to helipad center (this will naturally control heading)
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_ROI_LOCATION,
            0,
            0, 0, 0, 0,  # unused parameters
            helipad_lat,
            helipad_lon,
            0  # altitude above home
        )
        time.sleep(0.5)
        
        # Set position target without any heading command
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |  # Ignore yaw command
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE  # Ignore yaw rate
        )
        
        start_time = time.time()
        last_distance = None
        no_progress_count = 0
        
        while True:
            # Send position command (without heading)
            self.master.mav.set_position_target_global_int_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                lat_int,
                lon_int,
                alt,
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0,  # yaw (ignored due to type_mask)
                0   # yaw_rate (ignored due to type_mask)
            )
            
            # Get current position
            current_pos = self.get_current_position()
            if not current_pos:
                continue
                
            # Calculate distance to target
            distance = self.get_distance(
                current_pos['lat'], current_pos['lon'],
                lat, lon
            )
            
            # Check for progress
            if last_distance is not None:
                if abs(distance - last_distance) < 0.1:  # Less than 0.1m progress
                    no_progress_count += 1
                else:
                    no_progress_count = 0
                    
            last_distance = distance
            
            print(f"\rDistance to waypoint: {distance:.1f}m", end='')
            
            # Success condition
            if distance < 2:  # Within 2 meters
                print("\nWaypoint reached")
                return True
                
            # No progress condition
            if no_progress_count > 10:  # No progress for 10 iterations
                if distance < 5:  # If we're close enough, consider it successful
                    print("\nClose enough to waypoint")
                    return True
                print("\nVehicle not making progress to waypoint")
                return False
                
            time.sleep(0.5)
    
    def get_current_position(self):
        """Get current position of the vehicle."""
        try:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg:
                return {
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'alt': msg.alt / 1000.0,
                    'relative_alt': msg.relative_alt / 1000.0
                }
        except Exception as e:
            print(f"Error getting position: {e}")
        return None
    
    def get_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two points in degrees."""
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        d_lon = lon2 - lon1
        y = math.sin(d_lon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(d_lon)
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360
    
    def get_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two points in meters."""
        R = 6371000  # Earth radius in meters
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        d_lat = lat2 - lat1
        d_lon = lon2 - lon1
        a = (math.sin(d_lat/2) * math.sin(d_lat/2) +
             math.cos(lat1) * math.cos(lat2) *
             math.sin(d_lon/2) * math.sin(d_lon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        return R * c

    def set_heading(self, heading):
        """Set vehicle heading while maintaining position."""
        current_pos = self.get_current_position()
        if not current_pos:
            return False
        
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        
        lat_int = int(current_pos['lat'] * 1e7)
        lon_int = int(current_pos['lon'] * 1e7)
        
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            type_mask,
            lat_int,
            lon_int,
            current_pos['relative_alt'],
            0, 0, 0,  # velocity
            0, 0, 0,  # acceleration
            math.radians(heading),
            0  # yaw rate
        )
        
        # Wait for heading change
        time.sleep(2)
        return True 

    def set_final_heading(self, heading):
        """Set final heading while maintaining current position."""
        current_pos = self.get_current_position()
        if not current_pos:
            return False
        
        # Convert coordinates for mavlink
        lat_int = int(current_pos['lat'] * 1e7)
        lon_int = int(current_pos['lon'] * 1e7)
        
        # Set position target with specific heading
        type_mask = (
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
            mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
        )
        
        # Send command multiple times to ensure it takes effect
        for _ in range(3):
            self.master.mav.set_position_target_global_int_send(
                0,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                type_mask,
                lat_int,
                lon_int,
                current_pos['relative_alt'],
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                math.radians(heading),
                0  # yaw rate
            )
            time.sleep(1)
        
        # Wait for heading to stabilize
        time.sleep(2)
        return True 
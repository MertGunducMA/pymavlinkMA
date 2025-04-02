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
        
        if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            mode_id = self.master.mode_mapping()[mode]
        elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            if mode == "GUIDED":
                mode = "TAKEOFF"  # PX4 uses TAKEOFF mode instead of GUIDED
            mode_id = self.master.mode_mapping()[mode][1]
        else:
            print(f"Unsupported autopilot type: {self.autopilot_type}")
            return False
            
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
            
        # Set takeoff parameters based on autopilot type
        if self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
            takeoff_params = [0, 0, 0, 0, 0, 0, altitude_m]
        elif self.autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            starting_alt = msg.alt / 1000
            takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + altitude_m]
        else:
            print(f"Unsupported autopilot type: {self.autopilot_type}")
            return False
            
        # Send takeoff command
        print("Sending takeoff command...")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            takeoff_params[0],  # minimum pitch
            takeoff_params[1],
            takeoff_params[2],
            takeoff_params[3],
            takeoff_params[4],
            takeoff_params[5],
            takeoff_params[6]  # altitude
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
            print(f"Current altitude: {current_alt:.2f}m")
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
            print(f"Current altitude: {current_alt:.2f}m")
            if current_alt < 0.1:  # Less than 10cm from ground
                print("Landed successfully")
                break
            time.sleep(0.1)

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
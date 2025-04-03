#!/usr/bin/env python3
"""
Unified Mission Control for Oil Rig Landing
Handles takeoff, approach and alignment phases in a single connection.
"""

import sys
import time
from oilrig_guidance import OilRigGuidance
from helipad_alignment import HelipadAlignment

class MissionControl:
    def __init__(self, connection_string='udpin:localhost:14551'):
        """Initialize mission control with a single MAVLink connection."""
        print(f"\n=== Initializing Mission Control ===")
        print(f"Connecting to vehicle on {connection_string}")
        
        # Create a single connection instance
        self.guidance = OilRigGuidance(connection_string)
        # Share the same connection with alignment system
        self.alignment = HelipadAlignment(existing_connection=self.guidance.master)
        
    def execute_mission(self, helipad_lat, helipad_lon, helipad_heading, 
                       takeoff_alt=20, approach_alt=100, alignment_alt=20, 
                       offset_distance=200):
        """
        Execute complete mission including takeoff, approach and alignment phases.
        
        Args:
            helipad_lat (float): Helipad latitude in degrees
            helipad_lon (float): Helipad longitude in degrees
            helipad_heading (float): Helipad heading in degrees (0-360)
            takeoff_alt (float): Initial takeoff altitude in meters
            approach_alt (float): Approach phase altitude in meters
            alignment_alt (float): Alignment phase altitude in meters
            offset_distance (float): Distance to maintain from helipad in meters
        """
        try:
            mission_start_time = time.time()
            print("\n=== Starting Mission ===")
            print(f"Target Helipad: lat={helipad_lat:.6f}, lon={helipad_lon:.6f}")
            print(f"Helipad Heading: {helipad_heading:.1f}°")
            
            # Phase 0: Takeoff
            takeoff_start = time.time()
            print("\n=== Phase 0: Takeoff ===")
            if not self.guidance.takeoff(takeoff_alt):
                print("Takeoff failed")
                return False
                
            print("\nTakeoff complete")
            print("Transitioning to approach phase...")
            time.sleep(2)  # Short pause between phases
            
            # Phase 1: Approach
            print("\n=== Phase 1: Initial Approach ===")
            if not self.guidance.approach_helipad(helipad_lat, helipad_lon, 
                                                approach_alt, offset_distance):
                print("Approach phase failed")
                return False
                
            print("\nApproach phase complete")
            print("Stabilizing before alignment phase...")
            time.sleep(5)  # Longer pause to ensure stability
            
            # Ensure we're still in a good state
            current_pos = self.guidance.get_current_position()
            if not current_pos:
                print("Failed to get position before alignment")
                return False
                
            print(f"Current position before alignment:")
            print(f"Latitude: {current_pos['lat']:.6f}")
            print(f"Longitude: {current_pos['lon']:.6f}")
            print(f"Altitude: {current_pos['relative_alt']:.1f}m")
            
            # Phase 2: Alignment
            print("\n=== Phase 2: Helipad Alignment ===")
            if not self.alignment.execute_alignment(helipad_lat, helipad_lon, 
                                                  helipad_heading, alignment_alt):
                print("Alignment phase failed")
                return False
            
            print("\n=== Mission Complete ===")
            print("Vehicle in position for precision landing")
            return True
            
        except KeyboardInterrupt:
            print("\n=== Mission Cancelled by User ===")
            return False
        except Exception as e:
            print(f"\n=== Mission Failed: {str(e)} ===")
            return False

def main():
    # Mission parameters
    HELIPAD_LAT = 51.502051
    HELIPAD_LON = -0.770180
    HELIPAD_HEADING = 125.0
    TAKEOFF_ALT = 20.0
    APPROACH_ALT = 20.0
    ALIGNMENT_ALT = 20.0
    OFFSET_DISTANCE = 200.0
    
    try:
        # Create mission control instance
        mission = MissionControl()
        
        # Execute complete mission
        success = mission.execute_mission(
            HELIPAD_LAT,
            HELIPAD_LON,
            HELIPAD_HEADING,
            TAKEOFF_ALT,
            APPROACH_ALT,
            ALIGNMENT_ALT,
            OFFSET_DISTANCE
        )
        
        if success:
            print("\nMission completed successfully")
            print("Ready for precision landing phase")
        else:
            print("\nMission failed")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nError during mission: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 
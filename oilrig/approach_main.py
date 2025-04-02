#!/usr/bin/env python3
"""
Test script for oil rig approach functionality.
This script will command the vehicle to:
1. Take off to specified altitude
2. Approach oil rig location with 200m offset
3. Hold position at the approach point
"""

import sys
import time
from oilrig_guidance import OilRigGuidance

# Oil rig GPS coordinates (example - replace with actual coordinates)
OILRIG_LAT = 51.502051  # Replace with actual latitude
OILRIG_LON = -0.770180  # Replace with actual longitude
APPROACH_ALT = 20        # Altitude in meters (starting with lower altitude for testing)
OFFSET_DISTANCE = 200    # Distance to maintain from oil rig in meters

def main():
    # Connection settings
    connection_string = 'udpin:localhost:14551'
    print(f"\nConnecting to vehicle on {connection_string}")
    print("Make sure SITL is running")
    
    try:
        # Create guidance system instance
        print("Initializing guidance system...")
        guidance = OilRigGuidance(connection_string)
        
        # Start mission
        print("\n=== Starting Approach Test Mission ===")
        print(f"Target: Oil Rig at {OILRIG_LAT}, {OILRIG_LON}")
        print(f"Approach altitude: {APPROACH_ALT}m")
        print(f"Offset distance: {OFFSET_DISTANCE}m")
        
        try:
            # Step 1: Basic takeoff test first
            print("\n--- Step 1: Takeoff ---")
            if not guidance.takeoff(APPROACH_ALT):
                print("Takeoff failed")
                guidance.disarm()
                return
            
            print("Takeoff successful")
            print("Waiting 5 seconds to stabilize...")
            time.sleep(5)
            
            # Step 2: Approach oil rig
            print("\n--- Step 2: Approaching Oil Rig ---")
            if not guidance.approach_helipad(OILRIG_LAT, OILRIG_LON, 
                                          approach_alt=APPROACH_ALT,
                                          offset_distance=OFFSET_DISTANCE):
                print("Approach failed")
                guidance.land()
                guidance.disarm()
                return
            
            print("Successfully reached approach point")
            
            # Step 3: Hold position
            print("\n--- Step 3: Holding Position ---")
            print("Holding position at approach point. Press Ctrl+C to end mission.")
            
            # Monitor position while holding
            while True:
                current_pos = guidance.get_current_position()
                distance, bearing = guidance.get_distance_bearing(
                    current_pos['lat'],
                    current_pos['lon'],
                    OILRIG_LAT,
                    OILRIG_LON
                )
                print(f"\rHolding at {distance:.1f}m from oil rig, altitude: {current_pos['relative_alt']:.1f}m", end='')
                time.sleep(1)
                
        except Exception as e:
            print(f"\nMission failed: {e}")
            try:
                guidance.land()
                guidance.disarm()
            except:
                pass
            return
            
    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user")
        print("Initiating landing sequence...")
        try:
            guidance.land()
            guidance.disarm()
            print("Landed and disarmed successfully")
        except:
            print("Emergency landing sequence failed")
        
    except Exception as e:
        print(f"\nFailed to initialize guidance system: {e}")
        print("\nPlease check:")
        print("1. SITL is running")
        print("2. UDP port 14551 is available")
        sys.exit(1)

if __name__ == "__main__":
    main() 
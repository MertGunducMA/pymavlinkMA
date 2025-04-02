#!/usr/bin/env python3
"""
Main execution script for Oil Rig Helipad Guidance and Landing System.
"""

import sys
import time
from oilrig_guidance import OilRigGuidance

def main():
    try:
        # Create guidance system instance
        guidance = OilRigGuidance()
        
        # Basic test sequence
        try:
            # Takeoff to 20 meters
            if not guidance.takeoff(20):
                print("Takeoff failed. Disarming vehicle...")
                guidance.disarm()
                sys.exit(1)
            
            # Wait for 5 seconds at altitude
            print("Hovering for 5 seconds...")
            time.sleep(5)
            
            # Execute landing
            guidance.land()
            
            # Disarm after landing
            guidance.disarm()
            
            print("Mission completed successfully!")
            
        except Exception as e:
            print(f"Mission failed: {e}")
            guidance.disarm()
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        try:
            guidance.disarm()
        except:
            pass
        sys.exit(0)
    except Exception as e:
        print(f"Failed to initialize guidance system: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main() 
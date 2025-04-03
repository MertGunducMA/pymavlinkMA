#!/usr/bin/env python3
"""
Main script for helipad alignment phase.
This script executes the alignment maneuver to position the vehicle
at the correct heading while maintaining constant radius from the helipad.
"""

import sys
import time
from helipad_alignment import HelipadAlignment

# Test parameters (replace with actual values)
HELIPAD_LAT = 51.502051
HELIPAD_LON = -0.770180
HELIPAD_HEADING = 3.0  # degrees
ALIGNMENT_ALTITUDE = 20.0  # meters

def main():
    try:
        # Initialize alignment system
        print("Initializing helipad alignment system...")
        alignment = HelipadAlignment()
        
        # Execute alignment
        success = alignment.execute_alignment(
            HELIPAD_LAT,
            HELIPAD_LON,
            HELIPAD_HEADING,
            ALIGNMENT_ALTITUDE
        )
        
        if success:
            print("\nAlignment phase completed successfully")
            print("Ready for precision landing phase")
        else:
            print("\nAlignment phase failed")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
        sys.exit(0)
    except Exception as e:
        print(f"\nError during alignment: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()

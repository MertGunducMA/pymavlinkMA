# Oil Rig Helipad Guidance and Landing System

This project implements guidance and landing control for an autonomous helicopter approaching and landing on an oil rig helipad using MAVLink communication with ArduPilot SITL.

## Setup

1. Ensure you have Python 3.8+ installed
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```
3. Make sure ArduPilot SITL is running in WSL

## Project Structure

- `oilrig_guidance.py`: Main script for guidance and landing control
- `requirements.txt`: Project dependencies
- `README.md`: Project documentation

## Usage

1. Start ArduPilot SITL in WSL
2. Run the guidance script:
   ```bash
   python oilrig_guidance.py
   ```

## Features

- MAVLink communication with ArduPilot SITL
- Guidance control for helipad approach
- Landing sequence control 
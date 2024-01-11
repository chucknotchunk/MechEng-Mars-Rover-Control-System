# Mars Rover Control System

## Project Overview
The Mars Rover Control System is a sophisticated software suite designed to operate a rover with precise movement control. This system includes features for basic motion commands, PID control for stability, and advanced steering functionalities. It lays the foundation for future enhancements like sensor fusion and GUI-based control.

## Features
- **Basic Word Commands:** Control rover movement with simple commands like "forward 5" to move forward 5 meters.
- **PID Control:** Optimized PID parameters for stable and responsive motor control, minimizing overshoot and settling time.
- **Modular Code Design:** Highly modularized and generalized code for ease of maintenance and scalability.
- **Steering Functions:**
  - Initial tank steering (spot steering) implementation.
  - Planned advanced steering with sensor fusion and dynamic power distribution.

## Getting Started
1. **Initial Setup:**
   - Clone the repository to your local machine.
   - Ensure you have the Arduino IDE installed.
   - Connect your rover hardware as per the schematic provided in the `docs` folder.

2. **Configuration:**
   - Adjust the parameters in the code such as `WHEEL_CIRCUMFERENCE`, `PULSE_PER_REV`, and `GEAR_RATIO` to match your rover's hardware specifications.
   - Refer to the 'Configuration Guide' in the `docs` folder for detailed instructions.

3. **Upload the Code:**
   - Open the project in Arduino IDE.
   - Select the correct board and port.
   - Upload the code to your rover's microcontroller.

## Usage
- To control the rover, use the serial monitor in the Arduino IDE.
- Enter commands in the format "forward [distance]" or "backward [distance]".
- The rover will execute these commands, moving the specified distance.

## Development Roadmap
- **Steering Enhancements:** Implementation of 'drive while steering' functionality.
- **Sensor Fusion:** Integrating IMU with wheel encoders for precise navigation.
- **GUI Implementation:** Development of a user-friendly graphical interface for control.
- **Traction Control:** Addition of a dynamic power distribution system for enhanced traction.

## Acknowledgments
- Special thanks to Dr. Schwingshackl for guidance and feedback on the project.
- Contributors and testers who have helped refine and improve the system.

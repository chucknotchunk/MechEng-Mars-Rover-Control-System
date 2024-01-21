# Mars Rover Control System

## Project Overview
The Mars Rover Control System is a sophisticated software suite designed to operate a rover with precise movement control. This system includes features for basic motion commands, PID control for stability, and advanced steering functionalities. It lays the foundation for future enhancements like sensor fusion and GUI-based control.

## Features
- **Basic Word Commands:** Control rover movement with simple commands like "move 5" to move forward 5 meters.
- **PID Control:** Optimized PID parameters for stable and responsive motor control, minimizing overshoot and settling time.
- **Modular Code Design:** Highly modularized and generalized code for ease of maintenance and scalability.
- **Steering Functions:**
  - Initial skid steering implementation.
  - Planned advanced steering with sensor fusion and dynamic power distribution.

## Getting Started
1. **Initial Setup:**
   - Clone the repository to your local machine.
   - Ensure you have the Arduino IDE installed.
   - Connect your rover hardware as per the schematic provided in the `docs` folder.

### 2. **Configuration:**
   - Before uploading the code to the microcontroller, adjust the following parameters in the code to match your rover's hardware specifications:
     - `pulsePerRev`: The number of encoder pulses per revolution. Default is set to `11`.
     - `gearRatio`: The gear ratio of your rover's motor. Default is set to `103`.
     - `wheelRadius`: The radius of your rover's wheels. Default is set to `10` cm.
     - `axleTrack`: The distance between the center of the tires on each axle. Default is set to `1.2` meters.
     - `wheelBase`: The distance between the front and rear axle. Default is set to `1` meter.
   - Refer to the 'Configuration Guide' in the `docs` folder for detailed instructions on these parameters.


3. **Upload the Code:**
   - Open the project in Arduino IDE.
   - Select the correct board and port.
   - Upload the code to your rover's microcontroller.

## Usage

To control the rover, use the serial monitor in the Arduino IDE. The rover accepts several commands, each of which controls different aspects of its movement:

- **Moving Straight:** To move the rover straight, enter the command in the format `move [distance]`. Replace `[distance]` with the desired distance value. The rover will move straight for the specified distance. Distance can be positive for forward movement or negative for backward movement.

- **Turning:** To turn the rover, use the command `turn [angle] [radius]`. Replace `[angle]` with the desired turning angle (in degrees) and `[radius]` with the turning radius. Positive angles will turn the rover to the right, and negative angles will turn it to the left. Note that the radius should be a positive number.

- **Stopping:** To immediately stop the rover, enter the command `stop`. This command halts all rover movements and operations.

- **Resuming Movement:** After stopping, to resume rover operations, enter the command `resume`. This command allows the rover to accept and execute new movement commands.

Example Commands:
- Move forward 10 meters: `move 10`
- Move backward 5 meters: `move -5`
- Turn right 45 degrees with a radius of 2 meters: `turn 45 2`
- Turn left 30 degrees with a radius of 3 meters: `turn -30 3`
- Stop the rover: `stop`
- Resume the rover: `resume`

Ensure the rover is in a safe and clear area before sending movement commands. Monitor the rover's operation closely to prevent accidents or collisions.

## Development Roadmap

- **Steering Enhancements:** 
  - [*Implemented*] Implemented 'turn by radius' functionality, allowing the rover to turn with a specified angle and radius.

- **Sensor Fusion:** 
  - [*Planned*] Integrating an Inertial Measurement Unit (IMU) with wheel encoders to enhance navigation accuracy and stability.

- **GUI Implementation:** 
  - [*Planned*] Developing a graphical user interface (GUI) to provide a more intuitive and user-friendly control experience for the rover.

- **Traction Control:** 
  - [*Planned*] Adding a dynamic power distribution system to improve traction across different terrains, enhancing the rover's adaptability and performance.


## Acknowledgments
- Special thanks to Dr. Schwingshackl for guidance and feedback on the project.
- Contributors and testers who have helped refine and improve the system.

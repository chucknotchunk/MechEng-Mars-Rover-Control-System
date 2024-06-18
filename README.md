# Mars Rover Control System
![](https://i.postimg.cc/J094jyk1/Whats-App-Image-2024-03-24-at-22-07-13-a4611edf.jpg)
## Project Overview
The Mars Rover Control System is designed to coordinate the actions between onboard subsystems and includes a motion control system to direct the Rover in executing commanded movements. This system establishes a foundation for future enhancements, including sensor fusion and GUI-based control.

## Features
- **Basic Word Commands:** Control rover movement with simple commands like "move 5" to move forward 5 meters.
- **PID Control:** Optimized PID parameters for stable and responsive motor control, minimizing overshoot and settling time.
- **Modular Code Design:** Highly modularized and generalized code for ease of maintenance and scalability.
- **Steering Functions:**
  - Initial skid steering implementation.
  - Planned advanced steering with sensor fusion and dynamic power distribution.
- **Central Controller Integration:**
  - Centralized command processing to coordinate various subsystems.
  - Improved synchronization and communication between motion control, panel deployment, and sample collection systems.
  - Enhanced overall system integration and performance, ensuring cohesive operation of all rover functionalities.

## Getting Started
1. **Initial Setup:**
   - Clone the repository to your local machine.
   - Ensure you have the Arduino IDE installed.
   - Connect your rover hardware as per the guide provided in the `Docs` folder.

### 2. **Configuration:**
   - Before uploading the code to the microcontroller, adjust the following parameters in the code to match your rover's hardware specifications:
     - `pulsePerRev`: The number of encoder pulses per revolution. Default is set to `16`.
     - `gearRatio`: The gear ratio of your rover's motor. Default is set to `55`.
     - `wheelRadius`: The radius of your rover's wheels. Default is set to `10` cm.
     - `axleTrack`: The distance between the center of the tires on each axle. Default is set to `1.2` meters.
     - `wheelBase`: The distance between the front and rear axle. Default is set to `1` meter.

3. **Upload the Code:**
   - Open the project in Arduino IDE.
   - Select the correct board and port.
   - Upload the code to your rover's microcontroller.

## Usage

To control the rover, use the serial monitor in the Arduino IDE. The rover accepts several commands, each of which controls different aspects of its movement:

### Motion Control

- **Moving Straight:** To move the rover straight, enter the command in the format `move [distance]`. Replace `[distance]` with the desired distance value. The rover will move straight for the specified distance. Distance can be positive for forward movement or negative for backward movement.

- **Turning:** To turn the rover, use the command `turn [angle] [radius]`. Replace `[angle]` with the desired turning angle (in degrees) and `[radius]` with the turning radius. Positive angles will turn the rover to the left, and negative angles will turn it to the right. Note that the radius should be a positive number.

- **Stopping:** To immediately stop the rover, enter the command `stop`. This command halts all rover movements and operations.

- **Resuming Movement:** After stopping, to resume rover operations, enter the command `resume`. This command allows the rover to accept and execute new movement commands.

### Panel Deployment

- **Deploying the Panel:** To deploy the panel, enter the command `deploy panel`. This command will activate the panel deployment mechanism.

- **Retracting the Panel:** To retract the panel, enter the command `retract panel`. This command will retract the panel if it is currently deployed.

### Sample Collection

- **Starting Drilling:** To start the drilling process, enter the command `start drill`. This command will activate the drill if the rover is stationary and the panel is retracted.

- **Stopping Drilling:** To stop the drilling process, enter the command `stop drill`. This command will halt the drill operation.

### Example Commands:

#### Motion Control
- Move forward 10 meters: `move 10`
- Move backward 5 meters: `move -5`
- Turn right 45 degrees with a radius of 2 meters: `turn 45 2`
- Turn left 30 degrees with a radius of 3 meters: `turn -30 3`
- Stop the rover: `stop`
- Resume the rover: `resume`

#### Panel Deployment
- Deploy the panel: `deploy panel`
- Retract the panel: `retract panel`

#### Sample Collection
- Start drilling: `start drill`
- Stop drilling: `stop drill`

Ensure the rover is in a safe and clear area before sending movement commands. Monitor the rover's operation closely to prevent accidents or collisions.


## Development Roadmap

- **Steering Enhancements:** 
  - [*Implemented*] Implemented 'turn by radius' functionality, allowing the rover to turn with a specified angle and radius.

- **Central Controller:** 
  - [*Implemented*] Developing a central controller to coordinate various subsystems of the rover, improving overall system integration and performance.

- **GUI Implementation:** 
  - [*Planned*] Developing a graphical user interface (GUI) to provide a more intuitive and user-friendly control experience for the rover.

- **Sensor Fusion:** 
  - [*Planned*] Integrating an Inertial Measurement Unit (IMU) with wheel encoders to enhance navigation accuracy and stability.
  
- **Trajectory Planning:** 
  - [*Planned*] Implementing trajectory planning algorithms to enable the rover to navigate via waypoints autonomously.

## Acknowledgments
- Special thanks to Dr. Schwingshackl for guidance and feedback on the project.
- Contributors and testers who have helped refine and improve the system.

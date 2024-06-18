The MechEng Mars Rover currently has four subsystems, as shown in the figure below, each controlled by one Arduino, and a central controller is used to control the peripheral Arduinos via the I2C bus.
![](https://i.postimg.cc/fy94WFFL/Control-System-Schematics-V1-5-Page-1-2.png)
This section provides a detailed guide on the cable connections between the central controller and its subsystems.

**Required Components:**
- Central Controller (Arduino)
- Subsystems Controllers (Arduino)
- Jumper Wires 

**Detailed Steps:**
1. Connect each subsystem Arduino to the central controller's I2C bus using jumper wires:
   - Connect the SCL (A4) pin on the subsystem Arduino to the SCL (A4) pin on the central controller.
   - Connect the SDA (A5) pin on the subsystem Arduino to the SDA (A5) pin on the central controller.
2. For the motion control Arduino, connect the TX0 pin to the D2 pin on the central controller and the RX1 pin to the D3 pin on the central controller.

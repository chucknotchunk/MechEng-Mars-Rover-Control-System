#ifndef _COMMAND_H
#define _COMMAND_H

// Commands
void sendPanelCommand(byte slaveAddr, byte command) {
  Wire.beginTransmission(slaveAddr);
  Wire.write(command);
  Wire.endTransmission();
}

void sendSamplingCommand(byte slaveAddr, byte command) {
  Wire.beginTransmission(slaveAddr);
  Wire.write(command);
  Wire.endTransmission();
}

void sendMotionCommand(byte slaveAddr, String command) {
  Wire.beginTransmission(slaveAddr);
  Wire.write(command.c_str());
  Wire.endTransmission();
}

#endif
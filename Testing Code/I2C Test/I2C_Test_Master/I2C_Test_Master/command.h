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

#endif
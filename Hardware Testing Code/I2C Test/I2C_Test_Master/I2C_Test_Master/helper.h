#ifndef _HELPER_H
#define _HELPER_H

// Helper function to convert bytes to float
float bytesToFloat(byte* bytes, int offset) {
  float value;
  memcpy(&value, bytes + offset, sizeof(float));
  return value;
}

#endif
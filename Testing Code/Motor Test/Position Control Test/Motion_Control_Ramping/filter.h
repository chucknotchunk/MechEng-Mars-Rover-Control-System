#ifndef _FILTER_H
#define _FILTER_H

class LowPassFilter {
public:
  LowPassFilter() {
    vPrev = 0.0;
    vFilt = 0.0;
  }

  float update(float v) {
    vFilt = 0.96906992 * vFilt + 0.01546504 * v + 0.01546504 * vPrev;
    vPrev = v;
    return vFilt;
  }

private:
  float vPrev;  // Previous input value
  float vFilt;  // Filtered output
};

#endif
#ifndef _FILTER_H
#define _FILTER_H

class LowPassFilter {
public:
  LowPassFilter() {
    vPrev = 0.0;
    vFilt = 0.0;
  }

  float update(float v) {
    vFilt = 0.854 * vFilt + 0.0728 * v + 0.0728 * vPrev;
    vPrev = v;
    return vFilt;
  }

private:
  float vPrev;  // Previous input value
  float vFilt;  // Filtered output
};


#endif
#ifndef _HELPER_H
#define _HELPER_H

int mySign(float x) {
  // Returns -1 if x is negative, 1 if x is positive, and 0 if x is zero.
  if (x > 0) {
    return 1;
  } else if (x < 0) {
    return -1;
  } else {
    return 0;
  }
}

float findMaxValue(float arr[], int size) {
  // findMaxValue returns the maximum value in the input array
  // Input: arr - a numerical array
  //        size - the number of elements in the array
  // Output: maxVal - the maximum value in the array

  // Initialize maxVal with the first element of the array
  float maxVal = arr[0];

  // Loop through the array to find the maximum value
  for (int i = 1; i < size; i++) {
    if (arr[i] > maxVal) {
      maxVal = arr[i];
    }
  }

  return maxVal;
}

#endif
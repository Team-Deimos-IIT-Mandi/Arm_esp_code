#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <stdint.h>
#include <math.h>
#include <algorithm>

#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 3

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define abs(x) ((x)>0?(x):-(x))
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define delayMicroseconds(us)
#define delay(ms)

typedef uint32_t uint32;

inline unsigned long millis() { return 0; }

#endif

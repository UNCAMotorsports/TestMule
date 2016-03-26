// MuleThrottle.h

#ifndef _MULETHROTTLE_h
#define _MULETHROTTLE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class MuleThrottle
{
public:
    MuleThrottle();
    ~MuleThrottle();

    void init();

    uint16_t getSafeThrottle();
    uint16_t getUnsafeThrottle();
    uint16_t getThrottle(uint8_t);

    void setThrottleMin(uint16_t);
    void setThrottleMax(uint16_t);
    void setThrottleRange(uint16_t);

    uint16_t getThrottleMin();
    uint16_t getThrottleMax();
    uint16_t getThrottleRange();

private:
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint16_t throttleRange;

};

#endif


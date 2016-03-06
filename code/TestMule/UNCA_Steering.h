// UNCA_Steering.h

#ifndef _UNCA_STEERING_h
#define _UNCA_STEERING_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class UNCA_Steering{
public:
    UNCA_Steering();
    void init();
    double getSteeringAngle();
};


#endif
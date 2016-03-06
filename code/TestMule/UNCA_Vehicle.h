// UNCA_Vehicle.h

#ifndef _UNCA_VEHICLE_h
#define _UNCA_VEHICLE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class UNCA_VehicleClass
{
 protected:


 public:
	void init();
};

extern UNCA_VehicleClass UNCA_Vehicle;

#endif


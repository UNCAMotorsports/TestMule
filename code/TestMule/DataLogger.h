// DataLogger.h

#ifndef _DATALOGGER_h
#define _DATALOGGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif



class DataLogger
{
 private:
     uint8_t spiSpeed;
     uint8_t chipSelect;

     void logData();
     void writeHeader();

 public:
     DataLogger(uint8_t, uint8_t);
     void init();
     void begin();
     void logDataRecord();
};

#endif


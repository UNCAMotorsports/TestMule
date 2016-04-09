// DataLogger.h

#ifndef _DATALOGGER_h
#define _DATALOGGER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

struct entry{
    uint32_t time;
    uint16_t throttle;
    int16_t left;
    int16_t right;
    double steer;
    uint16_t speed;
};

class DataLogger
{
 private:
     char* fileName;
     uint8_t spiSpeed;
     uint8_t chipSelect;
     uint32_t numEntries;

     struct entry arrEntries[10];
     

 public:
     DataLogger(uint8_t, uint8_t);
     void init();
     void begin(const char* fileName);
     void logData();
     void addEntry(uint32_t time, uint16_t throttle, int16_t left, int16_t right, double steering, uint16_t speed);
     void writeHeader();
};

#endif


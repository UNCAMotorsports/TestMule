// DataLogger.h

#ifndef _DATALOGGER_h
#define _DATALOGGER_h

#include <SdFat.h>
#include "MuleDefines.h"

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

typedef struct __attribute__ ((packed)) {
    
    // Floats are 32-bit on Teensy 3.x with Teensyduino 1.6.x
    // Doubles are 64-bit on Teensy 3.x with Teensyduino 1.6.x

    uint8_t dataVersion = 1;    // Increment every time this struct changes

    uint32_t time;
    uint16_t throttle;
    int16_t left;
    int16_t right;
    float steer;
    uint16_t speed;

} mule_data_t;

class DataLogger
{
 private:
     char* fileName;
     uint8_t spiSpeed;
     uint8_t chipSelect;
     uint32_t numEntries;
     mule_data_t* buffer;

 public:
     DataLogger();
     void init();
     void logData();
     void addEntry(uint32_t time, uint16_t throttle, int16_t left, int16_t right, float steering, uint16_t speed);
     void writeHeader();
     void startBinLogger();
     void fastLog();
};

#endif


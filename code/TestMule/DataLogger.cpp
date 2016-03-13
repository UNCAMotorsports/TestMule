// 
// 
// 

#include "DataLogger.h"
#include "SdFat.h"

SdFat sd;
SdFile file;

DataLogger::DataLogger(uint8_t chipSelect, uint8_t spiSpeed)
{
    this->chipSelect = chipSelect;
    this->spiSpeed = spiSpeed;
}

void DataLogger::init()
{

}

void DataLogger::begin()
{
    if (!sd.begin(chipSelect, spiSpeed))
        sd.initErrorHalt();
}

void logData()
{

}

void writeHeader() {
}
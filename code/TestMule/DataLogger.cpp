// 
// 
// 

#include "DataLogger.h"
#include "SdFat.h"

#define error(s) sd.errorHalt(F(s))

SdFat sd;
SdFile file;

DataLogger::DataLogger(uint8_t chipSelect, uint8_t spiSpeed)
{
    this->chipSelect = chipSelect;
    this->spiSpeed = spiSpeed;
    numEntries = 0;
}

void DataLogger::init()
{

}

void DataLogger::begin(const char* fileName)
{
    if (!sd.begin(chipSelect, spiSpeed)) {
        sd.initErrorHalt();
    }
    if (!file.open(fileName, O_CREAT | O_WRITE)) {
        error("file.open");
    }
    writeHeader();
}

void DataLogger::addEntry(uint32_t time, int16_t left, int16_t right, int16_t steering, uint16_t speed){
    this->arrEntries[numEntries].time = time;
    this->arrEntries[numEntries].left = left;
    this->arrEntries[numEntries].right = right;
    this->arrEntries[numEntries].steer = steering;
    this->arrEntries[numEntries].speed = speed;
    numEntries++;
    if (numEntries >= 10){
        logData();
        if (!file.sync() || file.getWriteError()) {
            error("write error");
        }
    }

}

void DataLogger::logData()
{
    for (int i = 0; i < 10; i++)
    {
        file.printf("%d,%d,%d,%d,%d\n", arrEntries[i].time, arrEntries[i].left, arrEntries[i].right, arrEntries[i].steer, arrEntries[i].speed);
    }
    numEntries = 0;
}

void DataLogger::writeHeader() {
    file.printf(F("Millis,Left,Right,Steering Angle,Wheel Speed\n"));
    if (!file.sync() || file.getWriteError()) {
        error("write error");
    }
}
// 
// 
// 

#include "DataLogger.h"


#define error(s) sd.errorHalt(F(s))
#define SD_SPI_SPEED        (SPI_FULL_SPEED)

SdFat sd;
SdFile file;
SdBaseFile binFile;

// Number of data records in a block.
const uint16_t DATA_DIM = 512 / sizeof(mule_data_t);

//Compute fill so block size is 512 bytes.  FILL_DIM may be zero.
const uint16_t FILL_DIM = 512 - DATA_DIM*sizeof(mule_data_t);

// Maximum file size in blocks.
// The program creates a contiguous file with FILE_BLOCK_COUNT 512 byte blocks.
// This file is flash erased using special SD commands.
// 51430 entries is good for 3600 seconds of logging (60 minutes)
const uint32_t FILE_BLOCK_COUNT = 51430;

// max number of blocks to erase per erase call
uint32_t const ERASE_SIZE = 262144L;

uint32_t bgnBlock, endBlock, blockNum = 0;

struct block_t {
    mule_data_t data[DATA_DIM];
    uint8_t fill[FILL_DIM];
};

block_t block;

DataLogger::DataLogger()
{
    numEntries = 0;
}

void DataLogger::addEntry(uint32_t time, uint16_t throttle, int16_t left, int16_t right, float steering, uint16_t speed){
    block.data[blockNum].dataVersion = 1;
    block.data[blockNum].time = time;
    block.data[blockNum].throttle = throttle;
    block.data[blockNum].left = left;
    block.data[blockNum].right = right;
    block.data[blockNum].steer = steering;
    block.data[blockNum].speed = speed;
}

// Write the buffered data to the card
void DataLogger::fastLog(){
    if (blockNum == DATA_DIM - 1){
        if (!sd.card()->isBusy()){
            if (!sd.card()->writeData((uint8_t*)&block)){
                error("fast write failed");
            }
            blockNum = 0;
        }
        else
            Serial.println("Card BUSY!");
    }
    else
        blockNum++;
}

// Write the first line of the 
void DataLogger::writeHeader() {
    file.printf(F("Version,millis,throttle,Left,Right,Steering Angle,Wheel Speed\n"));
    if (!file.sync() || file.getWriteError()) {
        error("write error");
    }
}

void DataLogger::startBinLogger(){

#ifdef LOGGER_DEBUG
    Serial.print("Size of Struct: ");
    Serial.println(sizeof(mule_data_t));
    Serial.print("Data_DIM: ");
    Serial.println(DATA_DIM);
    Serial.print("FILL_DIM: ");
    Serial.println(FILL_DIM);
    Serial.print("Sizeof Block: ");
    Serial.println(sizeof(block_t));
    Serial.println();
#endif

    if (!sd.begin(CS_SD, SD_SPI_SPEED)) {
        sd.initErrorHalt();
    }

    int number = 0;
    char sName[80];

    // Find a filename that hasn't been used already
    do
    {
        sprintf(sName, "Mule_Data_%d.bin", number++);
    } while (sd.exists(sName));

    binFile.close();

    //binFile.dateTimeCallback(dateTime);

    if (!binFile.createContiguous(sd.vwd(), sName, 512 * FILE_BLOCK_COUNT)){
        error("createContiguous failed");
    }

    if (!binFile.contiguousRange(&bgnBlock, &endBlock)){
        error("contiguousRange failed");
    }

    // Use SdFat's internal buffer ( ???? )
    uint8_t* cache = (uint8_t*)sd.vol()->cacheClear();
    if (cache == 0) {
        error("cacheClear failed");
    }

    //binFile.dateTimeCallbackCancel();

    uint32_t bgnErase = bgnBlock;
    uint32_t endErase;
    while (bgnErase < endBlock) {
        endErase = bgnErase + ERASE_SIZE;
        if (endErase > endBlock) {
            endErase = endBlock;
        }
        if (!sd.card()->erase(bgnErase, endErase)) {
            error("erase failed");
        }
        bgnErase = endErase + 1;
    }


    // Start a multiple block write.
    if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT)) {
        error("writeBegin failed");
    }
}
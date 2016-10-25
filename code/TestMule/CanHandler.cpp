#include "canheader.h"

static uint32_t FLWheelSpeed = 0;
static uint32_t FRWheelSpeed = 0;
static uint32_t RLWheelSpeed = 0;
static uint32_t RRWheelSpeed = 0;

void handleMessage(uint32_t msgID){
    switch (msgID)
    {
    case CAN_MSG_HARDFAULT: handleHardFault(); break;
    case CAN_MSG_SOFTFAULT: handleSoftFault(); break;
    case CAN_MSG_EXITVEHICLE: handleExitVehicle(); break;
    default: Serial.print("No message with ID: 0x"); Serial.println(msg.id, HEX);
        break;
    }
}
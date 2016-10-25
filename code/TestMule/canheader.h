#ifndef CAN_ENUM_H
#define CAN_ENUM_H

enum unca_can_id {
    CAN_MSG_HARDFAULT = 0x000,
    CAN_MSG_SOFTFAULT = 0x001,
    CAN_MSG_EXITVEHICLE = 0x002,
    CAN_MSG_INTERLOCKOPEN = 0x007,
    CAN_MSG_PACKVOLTAGE = 0x008,
    CAN_MSG_PACKCURRENT = 0x009,
    CAN_MSG_PACKOVERUNDERVOLT = 0x00A,
    CAN_MSG_PACKOVERCURRENT = 0x00B,
    CAN_MSG_PACKSTATUS = 0x00C,
    CAN_MSG_PACKCELLTEMP = 0x00D,
    CAN_MSG_PACKAMBTEMP = 0x00E,
    CAN_MSG_MOTOROVERTEMP = 0x012,
    CAN_MSG_MOTOROVERSPEED = 0x013,
    CAN_MSG_MOTORTEMP = 0x014,
    CAN_MSG_MOTORRPM = 0x015,
    CAN_MSG_MOTORTORQUE = 0x016,
    CAN_MSG_MOTORPOWER = 0x017,
    CAN_MSG_NODESTART = 0x01C,
    CAN_MSG_NODEMODE = 0x01D,
    CAN_MSG_NODEID = 0x01E,
    CAN_MSG_NODENAME = 0x01F,
    CAN_MSG_NODESTATUS = 0x020,
    CAN_MSG_NODEGETCONFIG = 0x021,
    CAN_MSG_NODEUPDATECONFIG = 0x022,
    CAN_MSG_NODESOFTRESET = 0x023,
    CAN_MSG_NODEHALT = 0x024,
};

void handleMessage(enum unca_can_id);
uint8_t handleHardFault();
uint8_t handleSoftFault();
uint8_t handleExitVehicle();


#endif
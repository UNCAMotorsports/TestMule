// demo: CAN-BUS Shield, send data
#include "mcp_can.h"
#include "mcp_can_dfs.h"
#include <SPI.h>

MCP_CAN CAN0(8);                                      // Set CS to pin 10
int val, count;
long timer;

void setup()
{
	Serial.begin(115200);
	delay(1000);
	// init can bus, baudrate: 500k
	if (CAN0.begin(CAN_1000KBPS) == CAN_OK) Serial.print("can init ok!!\r\n");
	else Serial.print("Can init fail!!\r\n");
	val = 0;
	count = 0;
}

byte stmp[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
void loop()
{
	timer = millis();
	while (count < 1024){
		// send data:  id = 0x00, standrad flame, data len = 8, stmp: data buf
		for (int i = 0; i < 8; i++)
		{
			stmp[i] = random(0, 255);
		}
		Serial.print("ID: ");
		Serial.print(val, HEX);
		Serial.print("  Data: ");
		for (int i = 0; i < 8; i++)                // Print each byte of the data
		{
			if (stmp[i] < 0x10)                     // If data byte is less than 0x10, add a leading zero
			{
				Serial.print("0");
			}
			Serial.print(stmp[i], HEX);
			Serial.print(" ");
		}
		Serial.println();
		CAN0.sendMsgBuf(val++, 0, 8, stmp);
		delay(100);                       // send data per 100ms
		count++;
	}
	Serial.println(millis() - timer);
	Serial.println("Done!");

	while (true)
	{
		delay(1000);
	}
}

/*********************************************************************************************************
END FILE
*********************************************************************************************************/

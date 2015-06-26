
#include <SPI.h>
#include <Adafruit_DotStar.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

#define DEBUG 1

#define tpsPin A0
#define steerPotPin A1

#define leftEncPin 2
#define leftRPMOut 5

#define rightEncPin 3
#define rightRPMOut 6

#define leftEncInt 0
#define rightEncInt 1

volatile uint32_t leftCount;
volatile uint32_t rightCount;

uint16_t leftRPM = 0;
uint16_t rightRPM = 0;
uint32_t lastCalcTime;

int16_t tpsValue;
int16_t steerVal;

void setup()
{
	pinMode(2, INPUT);		// Left motor Encoder
	pinMode(3, INPUT);		// Right motor Encoder
	pinMode(SS, INPUT_PULLUP);
	pinMode(MISO, OUTPUT);

	Serial.begin(460800);
	delay(1000);
	
	attachInterrupt(leftEncInt, encPulseLeft, RISING);		// Attach output of left motor encoder here
	attachInterrupt(rightEncInt, encPulseRight, RISING);		// Attach output of right motor encoder here
	lastCalcTime = micros();

	SPCR |= _BV(SPE);
	SPI.attachInterrupt();
}

void loop()
{
	// Read throttle position sensor
	tpsValue = analogRead(tpsPin);

	// Read Steering Position Sensor
	steerVal = analogRead(steerPotPin);

	// Check encoder values & calculate RPM if necessary
	uint32_t time = micros();
	if ((leftCount >= 20 && rightCount >= 20) || ((time - lastCalcTime) > 250000))
	{
		leftRPM = (150000 * leftCount) / (time - lastCalcTime);
		leftCount = 0;
		rightRPM = (150000 * rightCount) / (time - lastCalcTime);
		rightCount = 0;
		lastCalcTime = time;

		analogWrite(leftRPMOut, constrain(map(leftRPM, 0, 4095, 0, 255), 0, 255));
		analogWrite(rightRPMOut, constrain(map(rightRPM, 0, 4095, 0, 255), 0, 255));
		/*Serial.print(leftRPM);
		Serial.print(' ');
		Serial.println(rightRPM);*/
	}

}

void encPulseLeft(){
	leftCount++;
}

void encPulseRight(){
	rightCount++;
}

ISR(SPI_STC_vect)
{
	byte val = 0;

	val = leftRPM >> 8;
	SPI.transfer(val);

	val = leftRPM;
	SPI.transfer(val);

	val = rightRPM >> 8;
	SPI.transfer(val);

	val = rightRPM;
	SPI.transfer(val);

	val = '\n';
	SPI.transfer(val);
}

float calcPhase(){
	// Calculate the speed difference between left & right wheels based on the steering geometry 
}
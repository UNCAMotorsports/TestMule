#include <SPI.h>
#include "DAC_MCP49xx.h"

#define CS_DAC0 10
#define CS_DAC1 9
#define THROTTLE_PIN A0
#define STEERING_PIN A1
#define LATCH_PIN 4
#define ENC_TO_RPM 150000

DAC_MCP49xx dac0(DAC_MCP49xx::MCP4921, CS_DAC0);
DAC_MCP49xx dac1(DAC_MCP49xx::MCP4921, CS_DAC1);

uint32_t lastTime, thisTime;
uint32_t leftRPM, rightRPM;
uint16_t leftThrottle, rightThrottle;
uint32_t throttlePos, steeringPos;

volatile uint32_t APulses;
volatile uint32_t BPulses;

double leftSteer;
double rightSteer;

void setup()
{
	pinMode(CS_DAC0, OUTPUT);
	pinMode(CS_DAC1, OUTPUT);
	pinMode(11, OUTPUT);
	pinMode(LATCH_PIN, OUTPUT);

	digitalWrite(CS_DAC0, HIGH);
	digitalWrite(CS_DAC1, HIGH);
	digitalWrite(LATCH_PIN, HIGH);

	pinMode(13, OUTPUT);
	digitalWrite(13, LOW);

	Serial.begin(115200);

	//attachInterrupt(digitalPinToInterrupt(10), pulseA, RISING);
	//attachInterrupt(digitalPinToInterrupt(12), pulseB, RISING);

	SPI.begin();

	dac0.setSPIDivider(SPI_CLOCK_DIV16);
	dac1.setSPIDivider(SPI_CLOCK_DIV16);

	

	analogReadRes(16);
	lastTime = micros();
	APulses = 0;
	BPulses = 0;
}


void loop()
{
	thisTime = micros();
	// By math, we get 100 pulses every 5ms at 3000 rpm.  That's 1/4 of a turn of the motors, which is great resolution.
	if (APulses >= 100 || BPulses >= 100 || (thisTime - lastTime >= 5000))
	{
		APulses *= ENC_TO_RPM;
		BPulses *= ENC_TO_RPM;

		// High Pass Filter, x = e^(-1/2) - e.g. it takes 2 samples for x to decay to 36.68% of its initial value
		leftRPM = (long)((double)(.60*leftRPM) + .40*APulses / (thisTime - lastTime));
		rightRPM = (long)((double)(.60*rightRPM) + .40*BPulses / (thisTime - lastTime));
		
		APulses = 0;
		BPulses = 0;
		lastTime = micros();
	}

	// Read Throttle && steering pots once every millisecond (+ .1ms / analogRead)
	if (micros() - lastTime >= 1000)
	{
		// Read the throttle and steering potentiometers, pass them through a simple filter.
		throttlePos = .60*throttlePos + .40*analogRead(THROTTLE_PIN);
		steeringPos = .60*steeringPos + .40*analogRead(STEERING_PIN);
		
		/*
		*  Generate a linear correlation to steering angle and motor speed as a ratio 0-1.
		*  This should be replaced with actual Ackermann steering values when implemented on the car.
		*/
		leftSteer = constrain((steeringPos) / 32768.0, 0, 1);
		rightSteer = constrain((65535 - (steeringPos)) / 32768.0, 0, 1);

		// Calculate the value to send to the DAC, multiplied by the steering ratio.
		leftThrottle = (double)throttlePos / 65536.0 * 4096 * leftSteer;
		rightThrottle = (double)throttlePos / 65536.0 * 4096 * rightSteer;

		Serial.print("leftThrottle: ");
		Serial.print(leftThrottle);
		Serial.print("\trightThrottle: ");
		Serial.println(rightThrottle);

		// Write to the DACs
		dac0.output(leftThrottle);
		dac1.output(rightThrottle);

		/*uint16_t out = (0 << 15) | (1 << 14) | (1 << 13) | (1 << 12) | (leftThrottle);
		digitalWriteFast(CS_DAC0, LOW);
		SPI.transfer(out >> 8);
		SPI.transfer(out & 0xFF);
		digitalWriteFast(CS_DAC0, HIGH);

		out = (0 << 15) | (1 << 14) | (1 << 13) | (1 << 12) | (rightThrottle);
		digitalWriteFast(CS_DAC1, LOW);
		SPI.transfer(out >> 8);
		SPI.transfer(out & 0xFF);
		digitalWriteFast(CS_DAC1, HIGH);*/

		// Quickly latch the DAC output by writing directly to the registers (Uno pin 6)
		digitalWriteFast(LATCH_PIN, LOW);
		digitalWriteFast(LATCH_PIN, HIGH);
	}
}

void pulseA(){
	APulses++;
}

void pulseB(){
	BPulses++;
}

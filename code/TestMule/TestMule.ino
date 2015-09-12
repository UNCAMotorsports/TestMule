#include <SPI.h>
#include "DAC_MCP49xx.h"

#define CS_DAC0 10
#define CS_DAC1 8
#define THROTTLE_PIN A2
#define STEERING_PIN A5
#define LATCH_PIN 6

DAC_MCP49xx dac0(DAC_MCP49xx::MCP4921, CS_DAC0);
DAC_MCP49xx dac1(DAC_MCP49xx::MCP4921, CS_DAC1);

uint32_t lastTime, thisTime;
uint32_t leftRPM, rightRPM;
uint32_t leftThrottle, rightThrottle;
uint32_t throttlePos, steeringPos;

volatile uint32_t APulses;
volatile uint32_t BPulses;

void setup()
{
	Serial.begin(115200);
	delay(1000);

	pinMode(2, INPUT);
	pinMode(3, INPUT);
	pinMode(LATCH_PIN, OUTPUT);

	digitalWrite(LATCH_PIN, HIGH);

	attachInterrupt(digitalPinToInterrupt(2), pulseA, RISING);
	attachInterrupt(digitalPinToInterrupt(3), pulseB, RISING);

	dac0.setSPIDivider(SPI_CLOCK_DIV4);
	dac1.setSPIDivider(SPI_CLOCK_DIV4);

	lastTime = micros();
}


void loop()
{
	thisTime = micros();
	// By math, we get 100 pulses every 5ms at 3000 rpm.  That's 1/4 of a turn of the motors, which is great resolution.
	if (APulses >= 100 || BPulses >= 100 || (thisTime - lastTime >= 5000))
	{
		APulses *= 150000;
		BPulses *= 150000;

		// High Pass Filter, x = e^(-1/2) - e.g. it takes 2 samples for x to decay to 36.68% of its initial value
		leftRPM = (long)((double)(.60*leftRPM) + .40*APulses / (thisTime - lastTime));
		rightRPM = (long)((double)(.60*rightRPM) + .40*BPulses / (thisTime - lastTime));

		lastTime = micros();
		APulses = 0;
		BPulses = 0;
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
		double leftSteer = constrain((steeringPos + 1) / 512.0, 0, 1);
		double rightSteer = constrain((1024 - (steeringPos + 1)) / 512.0, 0, 1);

		// Calculate the value to send to the DAC, multiplied by the steering ratio.
		leftThrottle = (double)throttlePos / 1024.0 * 4096 * leftSteer;
		rightThrottle = (double)throttlePos / 1024.0 * 4096 * rightSteer;

		// Write to the DACs
		dac0.output(leftThrottle);
		dac1.output(rightThrottle);

		// Quickly latch the DAC output by writing directly to the registers (Uno pin 6)
		PORTD &= _BV(LATCH_PIN);
		PORTD |= _BV(LATCH_PIN);
	}
}

void pulseA(){
	APulses++;
}

void pulseB(){
	BPulses++;
}

#include <SPI.h>
#include "DAC_MCP49xx.h"
#include <i2c_t3.h>



#define CS_DAC0			10
#define CS_DAC1			9
#define THROTTLE_PIN	A0
#define STEERING_PIN	A1
#define LATCH_PIN		4

#define ENC_TO_RPM		150000

#define LEFT_ENC_PIN	5
#define RIGHT_ENC_PIN	6

#define POLLING_TIME	1000  // 1ms
#define RPM_TIME		5000  // 5ms

// Comment or remove these definitions to stop respective debug code from being compiled
//#define DEBUG_THROTTLE
#define DEBUG_RPM

DAC_MCP49xx dac0(DAC_MCP49xx::MCP4921, CS_DAC0);
DAC_MCP49xx dac1(DAC_MCP49xx::MCP4921, CS_DAC1);

uint32_t lastTime, thisTime;
uint32_t leftRPM, rightRPM;
uint16_t leftThrottle, rightThrottle;
uint32_t throttlePos, steeringPos;

volatile uint32_t leftPulses;
volatile uint32_t rightPulses;

double leftSteer;
double rightSteer;

void setup()
{
	pinMode(CS_DAC0, OUTPUT);
	pinMode(CS_DAC1, OUTPUT);

	Wire.beginTransmission(8);

	pinMode(LATCH_PIN, OUTPUT);

	pinMode(LEFT_ENC_PIN, INPUT);
	pinMode(RIGHT_ENC_PIN, INPUT);

	digitalWrite(CS_DAC0, HIGH);
	digitalWrite(CS_DAC1, HIGH);
	digitalWrite(LATCH_PIN, HIGH);

#if defined(DEBUG_THROTTLE) || defined(DEBUG_RPM)

	Serial.begin(115200);

#endif // DEBUG

	attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), pulseLeft, RISING);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), pulseRight, RISING);

	SPI.begin();

	dac0.setSPIDivider(SPI_CLOCK_DIV16);
	dac1.setSPIDivider(SPI_CLOCK_DIV16);

	analogReadResolution(12);

	lastTime = micros();
	leftPulses = 0;
	rightPulses = 0;
}


void loop()
{
	thisTime = micros();
	// By math, we get 100 pulses every 5ms at 3000 rpm.  That's 1/4 of a turn of the motors, which is great resolution.
	if (leftPulses >= 100 || rightPulses >= 100 || (thisTime - lastTime >= RPM_TIME))
	{
		leftPulses *= ENC_TO_RPM;
		rightPulses *= ENC_TO_RPM;

		// High Pass Filter, x = e^(-1/2) - e.g. it takes 2 samples for x to decay to 36.68% of its initial value
		leftRPM = (long)((double)(.60*leftRPM) + .40*leftPulses / (thisTime - lastTime));
		rightRPM = (long)((double)(.60*rightRPM) + .40*rightPulses / (thisTime - lastTime));

#ifdef DEBUG_RPM
			Serial.print("leftRPM: ");
			Serial.print(leftRPM);
			Serial.print("\trightRPM: ");
			Serial.println(rightRPM);

#endif // DEBUG

		leftPulses = 0;
		rightPulses = 0;
		lastTime = micros();
	}

	// Read Throttle && steering pots once every millisecond (+ .1ms / analogRead)
	if (micros() - lastTime >= POLLING_TIME)
	{
		// Read the throttle and steering potentiometers, pass them through a simple filter.

		throttlePos = .60*throttlePos + .40*analogRead(THROTTLE_PIN);
		steeringPos = .60*steeringPos + .40*analogRead(STEERING_PIN);

#ifdef DEBUG_THROTTLE
		
		//Serial.printf("throttlePos: %.2lf\tSteeringPos: %.2lf\n", throttlePos, steeringPos);
#endif

		/*
		*  Generate a linear correlation to steering angle and motor speed as a ratio 0-1.
		*  This should be replaced with actual Ackermann steering values when implemented on the car.
		*/
		leftSteer = constrain((steeringPos) / 2048.0, 0, 1);
		rightSteer = constrain((4095 - (steeringPos)) / 2048.0, 0, 1);

		// Calculate the value to send to the DAC, multiplied by the steering ratio.
		leftThrottle = (double)throttlePos * leftSteer;
		rightThrottle = (double)throttlePos * rightSteer;

#ifdef DEBUG_THROTTLE

		Serial.print("leftThrottle: ");
		Serial.print(leftThrottle);
		Serial.print("\trightThrottle: ");
		Serial.println(rightThrottle);

#endif // DEBUG

		// Write to the DACs
		dac0.output(leftThrottle);
		dac1.output(rightThrottle);

		// Quickly latch the DAC output
		digitalWriteFast(LATCH_PIN, LOW);
		digitalWriteFast(LATCH_PIN, HIGH);
	}
}

void pulseLeft(){
	leftPulses++;
}

void pulseRight(){
	rightPulses++;
}

#include "UNCA_Steering.h"
#include <SPI.h>
#include "DAC_MCP49xx.h"
#include "FlexCAN.h"
#include <i2c_t3.h>

// Teensy's max and min macros use non-standard gnu extensions... these are simpler for integers etc.
#define simple_max(a,b) (((a)>(b)) ? (a) : (b))
#define simple_min(a,b) (((a)<(b)) ? (a) : (b))
#define simple_constrain(amt,low,high) (((amt)<(low)) ? (low) : ((amt > high) ? (high) : (amt)))

#define CS_FLASH            2
#define CS_DAC0		        7
#define CS_DAC1		        8
#define LATCH_PIN	        9
#define CS_SD               10

#define THROTTLE0_PIN	    A0
#define THROTTLE1_PIN	    A1
#define STEERING0_PIN       A2
#define STEERING1_PIN       A3

#define ENC_TO_RPM		    75000   // 800 ppr effective, is divided by POLLING TIME (in uS)

#define LEFT_ENC_PIN	    5
#define RIGHT_ENC_PIN	    6

#define POLLING_TIME	    5000  // 5ms

#define DIFFERENTIAL_MODE   0

// Comment or remove these definitions to stop respective debug code from being compiled
//#define DEBUG_THROTTLE
//#define DEBUG_RPM
//#define DEBUG_PROFILING

#if defined(DEBUG_THROTTLE) || defined(DEBUG_RPM) || defined(DEBUG_PROFILING)
#define DEBUG_CAR
#endif

DAC_MCP49xx dac0(DAC_MCP49xx::MCP4921, CS_DAC0);
DAC_MCP49xx dac1(DAC_MCP49xx::MCP4921, CS_DAC1);

uint32_t lastTime, thisTime;

uint32_t omega_left;
uint32_t omega_right;
uint32_t omega_vehicle;

uint16_t throttleMin;
uint16_t throttleMax;
uint16_t throttleRange;
uint16_t requestedThrottle;
uint16_t leftThrottle;
uint16_t rightThrottle;

uint16_t steeringLeft;
uint16_t steeringRight;
uint16_t steeringCenter;

volatile uint32_t leftPulses;
volatile uint32_t rightPulses;

void setup()
{
    // Set pin modes
    pinMode(CS_FLASH, OUTPUT);
    pinMode(CS_DAC0, OUTPUT);
    pinMode(CS_DAC1, OUTPUT);
    pinMode(CS_SD, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);

    pinMode(LEFT_ENC_PIN, INPUT);
    pinMode(RIGHT_ENC_PIN, INPUT);

    digitalWrite(CS_FLASH, HIGH);
    digitalWrite(CS_DAC0, HIGH);
    digitalWrite(CS_DAC1, HIGH);
    digitalWrite(CS_SD, HIGH);

    digitalWrite(LATCH_PIN, LOW);   // LOW if you want the DAC values to change immediately.

    // Attach functions to interrupts for the encoders
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), pulseLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), pulseRight, CHANGE);

#ifdef DEBUG_CAR
    Serial.begin(115200);
    delay(1000);
#endif // DEBUG

    // Set Up DACs
    dac0.setSPIDivider(SPI_CLOCK_DIV8);
    dac1.setSPIDivider(SPI_CLOCK_DIV8);
    dac0.setPortWrite(false);
    dac1.setPortWrite(false);
    dac0.output(0);
    dac1.output(0);

    // Set up ADCs
    analogReadResolution(12);
    analogReadAveraging(8);

    
    throttleMin = getThrottle(THROTTLE0_PIN);      // Sample the throttle pots, set that value as minimum
    throttleRange = 1500;                                                           // Guesstimate max throttle
    throttleMax = throttleMin + throttleRange;

#ifdef DEBUG_THROTTLE
    Serial.printf("Throttle Min:\t%d\n", throttleMin);
    delay(1000);
    Serial.printf("Throttle Max:\t%d\n", throttleMax);
#endif

    // Take a first time reading
    lastTime = micros();
}


void loop()
{
    if ((micros() - lastTime) >= POLLING_TIME)
    {
        lastTime = micros();
        omega_left = leftPulses*ENC_TO_RPM / POLLING_TIME;

        omega_right = rightPulses*ENC_TO_RPM / POLLING_TIME;
        omega_vehicle = (simple_max(omega_left, omega_right) + simple_min(omega_left, omega_right)) / 2;

#ifdef DEBUG_RPM
        Serial.printf("Left pulses: %d\tRightPulses: %d\n", leftPulses, rightPulses);
        Serial.printf("Left RPM: %d\tRight RPM: %d\n", omega_left, omega_right);
#endif
        leftPulses = 0;
        rightPulses = 0;
        requestedThrottle = getThrottle(THROTTLE0_PIN);    // Safe throttle will need a better algorithm to handle noise
        requestedThrottle = simple_constrain(requestedThrottle, throttleMin, throttleMax);
        requestedThrottle -= throttleMin;
        requestedThrottle = requestedThrottle / (double)throttleRange * 4095;

        if (requestedThrottle < 75)     // Filter the lowest values so the car doesn't crawl
            requestedThrottle = 0;

#ifdef DEBUG_THROTTLE
        Serial.printf("\tRequested: %d\n", requestedThrottle);
#endif
        switch (DIFFERENTIAL_MODE)
        {
        case 0:
            leftThrottle = requestedThrottle;
            rightThrottle = requestedThrottle;
            break;
        }
        // Write to the DACs
        dac0.output(leftThrottle);
        dac1.output(rightThrottle);

#ifdef DEBUG_PROFILING
        Serial.println(micros()-lastTime);
#endif // DEBUG_PROFILING

    }
}

void pulseLeft(){
    leftPulses++;
}

void pulseRight(){
    rightPulses++;
}


// Just get the average of the two throttles
int16_t getUnsafeThrottle()
{
    uint16_t throttlePot0;
    uint16_t throttlePot1;
    uint16_t throttle0;
    uint16_t throttle1;

    throttlePot0 = analogRead(THROTTLE0_PIN);
    throttlePot1 = analogRead(THROTTLE1_PIN);

#ifdef DEBUG_THROTTLE
    Serial.print("Throttle 0:\t");
    Serial.print(throttlePot0);
    Serial.print("\tThrottle 1:\t");
    Serial.print(throttlePot1);
#endif

    // Constrain throttleMin < throttle < throttleMax
    throttle0 = simple_constrain(throttlePot0, throttleMin, throttleMax);
    throttle1 = simple_constrain(throttlePot1, throttleMin, throttleMax);

    return (throttle0 + throttle1) / 2;
}

// Get a single throttle value
int16_t getThrottle(uint8_t throttlePin)
{
    uint16_t throttlePot = 0;
    throttlePot = analogRead(throttlePin);

#ifdef DEBUG_THROTTLE
    Serial.printf("Throttle %d: ", throttlePin);
    Serial.print(throttlePot);
#endif

    if (throttlePot > 3684 || throttlePot < 410){
        Serial.printf("\nWarning:  Throttle out of range: %d", throttlePot);
        return -1;
    }

    return throttlePot;

}


// Check for plausibility and agreement, otherwise return -1
int16_t getSafeThrottle()
{
    uint16_t throttlePot0;
    uint16_t throttlePot1;
    uint16_t throttle0;
    uint16_t throttle1;

    throttlePot0 = analogRead(THROTTLE0_PIN);
    throttlePot1 = analogRead(THROTTLE1_PIN);

#ifdef DEBUG_THROTTLE
    Serial.print("Throttle 0:\t");
    Serial.print(throttlePot0);
    Serial.print("\tThrottle 1:\t");
    Serial.print(throttlePot1);
#endif

    if (throttlePot0 > 3684 || throttlePot0 < 410){
        Serial.printf("Warning:  Throttle 0 out of range: %d", throttlePot0);
        return -1;
    }
    else if (throttlePot1 > 3684 || throttlePot1 < 410)
    {
        Serial.printf("Warning:  Throttle 1 out of range: %d", throttlePot1);
        return -1;
    }

    throttle0 = ((throttlePot0 < throttleMin) ? throttleMin : ((throttlePot0 > throttleMax) ? throttleMax : throttlePot0));
    throttle1 = ((throttlePot1 < throttleMin) ? throttleMin : ((throttlePot1 > throttleMax) ? throttleMax : throttlePot1));

    if ((simple_max(throttle0, throttle1) - simple_min(throttle0, throttle1)) > 410)
    {
        Serial.printf("Warning:  Throttle Mismatch!");
        return -1;
    }

    return (throttle0 + throttle1) / 2;     // Return average of the two throttles
}

uint16_t getSteeringAngle()
{
    uint16_t steeringPot0;

    steeringPot0 = analogRead(STEERING0_PIN);

    return steeringPot0;
}
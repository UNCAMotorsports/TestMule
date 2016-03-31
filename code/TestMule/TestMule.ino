/* UNC Asheville Motorsports 2016 test mule code
*/


// Standard Headers
#include "MuleThrottle.h"
#include <SPI.h>
#include <FlexCAN.h>
#include <kinetis_flexcan.h>
#include <i2c_t3.h>

// Non-standard Headers
#include "DAC_MCP49xx.h"
#include "DataLogger.h"
#include "Vehicle_Stats.h"

// SDFatLib Headers
#include <SdFat.h>

#if defined(DEBUG_THROTTLE) || defined(DEBUG_RPM) || defined(DEBUG_PROFILING)
#define DEBUG_CAR
#endif

DAC_MCP49xx dac0(DAC_MCP49xx::MCP4921, CS_DAC0);
DAC_MCP49xx dac1(DAC_MCP49xx::MCP4921, CS_DAC1);

DataLogger sdLogger(CS_SD, SPI_CLOCK_DIV8);
MuleThrottle throttle;

uint32_t lastTime, thisTime;

int32_t omega_left;
int32_t omega_right;
int32_t omega_vehicle;

int16_t requestedThrottle;
int16_t leftThrottle;
int16_t rightThrottle;

int16_t steeringLeft;
int16_t steeringRight;
int16_t steeringCenter;

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
    analogReadAveraging(4);

    throttle.init();

#ifdef DEBUG_THROTTLE
    Serial.printf("Throttle Min:\t%d\n", throttle.getThrottleMin());
    delay(1000);
    Serial.printf("Throttle Max:\t%d\n", throttle.getThrottleMax());
#endif

    //sdLogger.begin();

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


        requestedThrottle = throttle.getThrottle(THROTTLE0_PIN);    // Safe throttle will need a better algorithm to handle noise
        requestedThrottle = simple_constrain(requestedThrottle, throttle.getThrottleMin(), throttle.getThrottleMax());
        requestedThrottle -= throttle.getThrottleMin();
        requestedThrottle = requestedThrottle / (double)throttle.getThrottleRange() * 4095;

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
        case 1:
            double steerAngle = getSteeringAngle();
            rightThrottle = requestedThrottle + requestedThrottle * .5 * TRACK_TO_WHEEL * steerAngle;
            leftThrottle = requestedThrottle - requestedThrottle * .5 * TRACK_TO_WHEEL * steerAngle;
            if (rightThrottle > 4095){
                double ratio = 4095.0 / rightThrottle;
                rightThrottle *= ratio;
                leftThrottle *= ratio;
            }
            rightThrottle = simple_constrain(rightThrottle, 0, 4095);
            leftThrottle = simple_constrain(leftThrottle, 0, 4095);
            break;
        }
#ifdef DEBUG_THROTTLE
        Serial.printf("\tLeft Throttle: %d\tRight Throttle: %d\n",leftThrottle,rightThrottle);
#endif
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

double getSteeringAngle()
{
    uint16_t steeringPot0;
    double steerAngle;

    steeringPot0 = analogRead(STEERING0_PIN);
    steerAngle = (steeringPot0 - steeringCenter) * RAD_PER_VAL;

    return steerAngle;
}

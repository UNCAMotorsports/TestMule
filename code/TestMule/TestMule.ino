/* UNC Asheville Motorsports 2016 test mule code */

#include <i2c_t3.h>
#include <Arduino.h>
#include "MuleDefines.h"
#include "DAC_MCP49xx.h"
#include "DataLogger.h"
#include "MuleThrottle.h"


/* ---------------------------------------------------------------------------- +
 *
 *      Set up the multi-rate main loop timer
 *
 * ---------------------------------------------------------------------------- */
#define TIMER_RATE          (1000)                          // Check the timer every 1 millisecond
#define RPM_RATE            (TIMER_RATE / 200)             // How often to check if we've stopped getting RPM readings
#define THROTTLE_RATE       (TIMER_RATE / 200)             // Read throttle at 200Hz
#define STEERING_RATE       (TIMER_RATE / 200)             // Read steering at 200Hz
#define LOGGING_RATE        (TIMER_RATE / 200)             // Create data entries at 200Hz (10-entry FIFO)

bool rpm_flag = false;
bool throttle_flag = false;
bool steering_flag = false;
bool logging_flag = false;

IntervalTimer loopTimer;
uint32_t timer = 0;

// Runs in an interrupt and sets the flags for our multi-rate main loop
void multiRateISR(){
    
    timer++;

    if (timer % THROTTLE_RATE == 0) { throttle_flag = true; }
    if (timer % STEERING_RATE == 0) { steering_flag = true; }
    if (timer % RPM_RATE == 0)      { rpm_flag = true; }
    if (timer % LOGGING_RATE == 0)  { logging_flag = true; }
    if (timer >= TIMER_RATE)        { timer = 0; }
}
/* ---------------------------------------------------------------------------- */

DAC_MCP49xx dac0(DAC_MCP49xx::MCP4921, CS_DAC0);
DAC_MCP49xx dac1(DAC_MCP49xx::MCP4921, CS_DAC1);

DataLogger sdLogger;
MuleThrottle throttle;
int16_t leftThrottle = 0;
int16_t rightThrottle = 0;
uint16_t requestedThrottle = 0;

uint32_t lastTime;


// Set up variables for tracking RPM
uint32_t lastLeftTime;
uint32_t lastRightTime;
volatile uint32_t numLeftPulses = 0;
volatile uint32_t numRightPulses = 0;
uint32_t leftRPM = 0;
uint32_t rightRPM = 0;
float omega_vehicle;

float steerAngle;

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

    // Set the default states for the various pins we're using
    digitalWriteFast(CS_FLASH, HIGH);
    digitalWriteFast(CS_DAC0, HIGH);
    digitalWriteFast(CS_DAC1, HIGH);
    digitalWriteFast(CS_SD, HIGH);

    digitalWriteFast(LATCH_PIN, LOW);   // LOW if you want the DAC values to change immediately.

    // Attach functions to interrupts for the encoders
    attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), pulseLeft, CHANGE);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), pulseRight, CHANGE);

    // Start Serial Communications with a host computer
    Serial.begin(115200);
    delay(1000);

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

    // Initialize the throttle sensor
    throttle.init();

#ifdef DEBUG_THROTTLE
    Serial.printf("Throttle Min:\t%d\n", throttle.getThrottleMin());
    delay(1000);
    Serial.printf("Throttle Max:\t%d\n", throttle.getThrottleMax());
#endif

    // Start datalogging
    sdLogger.startBinLogger();

    loopTimer.begin(multiRateISR, TIMER_RATE);        // Start the main loop timer

    lastLeftTime = micros();
    lastRightTime = micros();
    lastTime = micros();
}

/* ---------------------------------------------------------------------------- +
*       Main Loop
*  ---------------------------------------------------------------------------- */
void loop()
{

#ifdef DEBUG_PROFILING
    uint16_t profiler = micros();
#endif

    if (rpm_flag) 
    {
        rpmTask();
        rpm_flag = false;
    }
    else if (steering_flag) 
    {
        steeringTask();
        steering_flag = false;
    }
    else if (throttle_flag) 
    {
        throttleTask();
        throttle_flag = false;
    }
    else if (logging_flag)
    {
        // Add an entry to the logging buffer
        sdLogger.addEntry(micros(), requestedThrottle, leftThrottle, rightThrottle, steerAngle, rightRPM);
        sdLogger.fastLog();
        logging_flag = false;
    }

#ifdef DEBUG_PROFILING
    profiler = micros() - profiler;
    Serial.print("Loop Time: ");
    Serial.println(profiler);
#endif
}
/* ---------------------------------------------------------------------------- */

/* ---------------------------------------------------------------------------- +
*       Check for RPM timeout (where the wheel has stopped)
*  ---------------------------------------------------------------------------- */
void rpmTask(){

#ifdef DEBUG_PROFILING
    uint16_t profiler = micros();
#endif

    leftRPM = numLeftPulses * ENC_TO_RPM / (micros() - lastLeftTime);
    rightRPM = numRightPulses * ENC_TO_RPM / (micros() - lastRightTime);
    numLeftPulses = 0;
    numRightPulses = 0;
    lastLeftTime = micros();
    lastRightTime = micros();

    //omega_vehicle = (simple_max(leftRPM, rightRPM) + simple_min(leftRPM, rightRPM)) / 2.0;

#ifdef DEBUG_PROFILING
    profiler = micros() - profiler;
    Serial.print("RPM Time: ");
    Serial.println(profiler);
#endif

#ifdef DEBUG_RPM
    Serial.printf("Right RPM: %d\n", rightRPM);
#endif
}
/* ---------------------------------------------------------------------------- */


/* ---------------------------------------------------------------------------- +
*   Find the tangent of the steering angle
* ---------------------------------------------------------------------------- */
void steeringTask(){
    uint16_t steeringPot0;
    float steerAngle;

    steeringPot0 = analogRead(STEERING0_PIN);
    steerAngle = (steeringPot0 - STEERING_CENTER) * RAD_PER_VAL;

#ifdef DEBUG_STEERING

    Serial.printf("Raw Steering: %d\tSteer Angle: %0.2f\n", steeringPot0, steerAngle);
#endif

    steerAngle = tan(radians(steerAngle));
}
/* ---------------------------------------------------------------------------- */


/* ---------------------------------------------------------------------------- +
*   Poll the throttle sensor and calculate the differential action to each wheel
* ---------------------------------------------------------------------------- */
void throttleTask(){

#ifdef DEBUG_PROFILING
    uint16_t profiler = micros();
#endif

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

        rightThrottle = requestedThrottle + requestedThrottle * .5 * TRACK_TO_WHEEL * steerAngle;
        leftThrottle = requestedThrottle - requestedThrottle * .5 * TRACK_TO_WHEEL * steerAngle;

#ifdef DEBUG_THROTTLE
        Serial.printf("Delta: %f\n", requestedThrottle * .5 * TRACK_TO_WHEEL * steerAngle);
#endif
        double ratio = 1.0;
        if (rightThrottle > 4095){
            ratio = 4095.0 / rightThrottle;
        }
        else if (rightThrottle > 4095){
            ratio = 4095.0 / leftThrottle;
        }

        rightThrottle *= ratio;
        leftThrottle *= ratio;

        // The throttles should already be constrained by the above calculation, but just to make sure...
        rightThrottle = simple_constrain(rightThrottle, 0, 4095);
        leftThrottle = simple_constrain(leftThrottle, 0, 4095);
        break;
    }

#ifdef DEBUG_THROTTLE
    Serial.printf("Left Throttle: %d\tRight Throttle: %d\n", leftThrottle, rightThrottle);
#endif
    // Write to the DACs
    dac0.output(leftThrottle);
    dac1.output(rightThrottle);

#ifdef DEBUG_PROFILING
    profiler = micros() - profiler;
    Serial.print("Throttle Time: ");
    Serial.println(profiler);
#endif
}
/* ---------------------------------------------------------------------------- */

// Called when we get a wheel encoder pulse from the left
void pulseLeft(){
    numLeftPulses++;
}

// Called when we get a wheel encoder pulse from the right
void pulseRight(){
    numRightPulses++;
}
// 
// 
// 

#include "MuleThrottle.h"
#include "MuleDefines.h"


MuleThrottle::MuleThrottle()
{
}

MuleThrottle::~MuleThrottle()
{
}

void MuleThrottle::init()
{
    throttleMin = getThrottle(THROTTLE0_PIN);      // Sample the throttle pots, set that value as minimum
    throttleRange = 1450;                                   // Guesstimate max throttle
    throttleMax = throttleMin + throttleRange;
}



// Get a single throttle value
int16_t MuleThrottle::getThrottle(uint8_t throttlePin)
{
    int16_t throttlePot = 0;
    throttlePot = analogRead(throttlePin);

#ifdef DEBUG_THROTTLE
    Serial.printf(F("Throttle %d: "), throttlePin);
    Serial.print(throttlePot);
#endif

    if (throttlePot > 3684 || throttlePot < 410){
        Serial.printf(F("Warning:  Throttle out of range: %d"), throttlePot);
        return -1;
    }

    return throttlePot;

}

// Just get the average of the two throttles
int16_t MuleThrottle::getUnsafeThrottle()
{
    uint16_t throttlePot0;
    uint16_t throttlePot1;
    int16_t throttle0;
    int16_t throttle1;

    throttlePot0 = analogRead(THROTTLE0_PIN);
    throttlePot1 = analogRead(THROTTLE1_PIN);

#ifdef DEBUG_THROTTLE
    Serial.print(F("Throttle 0:\t"));
    Serial.print(throttlePot0);
    Serial.print(F("\tThrottle 1:\t"));
    Serial.print(throttlePot1);
#endif

    // Constrain throttleMin < throttle < throttleMax
    throttle0 = simple_constrain(throttlePot0, throttleMin, throttleMax);
    throttle1 = simple_constrain(throttlePot1, throttleMin, throttleMax);

    return (throttle0 + throttle1) / 2;
}

int16_t MuleThrottle::getSafeThrottle()
{
    int16_t throttlePot0;
    int16_t throttlePot1;
    int16_t throttle0;
    int16_t throttle1;

    throttlePot0 = analogRead(THROTTLE0_PIN);
    throttlePot1 = analogRead(THROTTLE1_PIN);

#ifdef DEBUG_THROTTLE
    Serial.print(F("Throttle 0:\t"));
    Serial.print(throttlePot0);
    Serial.print(F("\tThrottle 1:\t"));
    Serial.print(throttlePot1);
#endif

    if (throttlePot0 > 3684 || throttlePot0 < 410){
        Serial.printf(F("Warning:  Throttle 0 out of range: %d"), throttlePot0);
        return -1;
    }
    else if (throttlePot1 > 3684 || throttlePot1 < 410)
    {
        Serial.printf(F("Warning:  Throttle 1 out of range: %d"), throttlePot1);
        return -1;
    }

    throttle0 = ((throttlePot0 < throttleMin) ? throttleMin : ((throttlePot0 > throttleMax) ? throttleMax : throttlePot0));
    throttle1 = ((throttlePot1 < throttleMin) ? throttleMin : ((throttlePot1 > throttleMax) ? throttleMax : throttlePot1));

    if ((simple_max(throttle0, throttle1) - simple_min(throttle0, throttle1)) > 410)
    {
        Serial.printf(F("Warning:  Throttle Mismatch!"));
        return -1;
    }

    return (throttle0 + throttle1) / 2;     // Return average of the two throttles
}

void MuleThrottle::setThrottleMin(uint16_t nVal)
{
    this->throttleMin = nVal;
}

void MuleThrottle::setThrottleMax(uint16_t nVal)
{
    this->throttleMax = nVal;
}

void MuleThrottle::setThrottleRange(uint16_t nVal)
{
    this->throttleRange = nVal;
}

uint16_t MuleThrottle::getThrottleMin()
{
    return this->throttleMin;
}

uint16_t MuleThrottle::getThrottleMax()
{
    return this->throttleMax;
}

uint16_t MuleThrottle::getThrottleRange()
{
    return this->throttleRange;
}
/*
* File:    Vehicle_stats.h
* Purpose: All of the vehicle's Defines
*/


// Comment or remove these definitions to stop respective debug code from being compiled
//#define DEBUG_THROTTLE
//#define DEBUG_RPM
//#define DEBUG_STEERING
//#define DEBUG_PROFILING


// Teensy's max and min macros use non-standard gnu extensions... these are simpler for integers etc.
#define simple_max(a,b) (((a)>(b)) ? (a) : (b))
#define simple_min(a,b) (((a)<(b)) ? (a) : (b))
#define simple_constrain(amt,low,high) (((amt)<(low)) ? (low) : ((amt > high) ? (high) : (amt)))

#define CS_FLASH            2
#define CS_DAC0		        8
#define CS_DAC1		        7
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

#define WHEELBASE_IN        72.0      // In Inches
#define REAR_TRACK_IN       60.0      // In inches
#define TRACK_TO_WHEEL      (REAR_TRACK_IN/WHEELBASE_IN)
#define MASS_KG             272.2   // In KG
#define MASS_LBF            600     // In lb

#define RAD_PER_VAL         .05

#define DIFFERENTIAL_MODE   1
// Section:  Defines

// Comment or remove these definitions to stop respective debug code from being compiled
//#define DEBUG_THROTTLE
//#define DEBUG_RPM
//#define DEBUG_STEERING
//#define DEBUG_PROFILING
//#define LOGGER_DEBUG
//#define LOGGING

// Teensy's max and min macros use non-standard gnu extensions... these are simpler for integers etc.
#define simple_max(a,b) (((a)>(b)) ? (a) : (b))
#define simple_min(a,b) (((a)<(b)) ? (a) : (b))
#define simple_constrain(amt,low,high) (((amt)<(low)) ? (low) : ((amt > high) ? (high) : (amt)))

#define CS_FLASH            (2)
#define CS_DAC0		        (8)
#define CS_DAC1		        (7)
#define LATCH_PIN	        (9)
#define CS_SD               (10)

#define THROTTLE0_PIN	    (A0)
#define THROTTLE1_PIN	    (A1)
#define STEERING0_PIN       (A2)
#define STEERING1_PIN       (A3)

#define ENC_TO_RPM_RIGHT    (50000)    // 800 ppr effective
#define ENC_TO_RPM_LEFT     (50000)
#define LEFT_ENC_PIN	    (5)
#define RIGHT_ENC_PIN	    (6)
#define WHEELBASE_IN        (72.0)      // In Inches
#define REAR_TRACK_IN       (42.0)      // In inches
#define TRACK_TO_WHEEL      (REAR_TRACK_IN/WHEELBASE_IN)

#define RAD_PER_VAL         (.03)

#define DIFFERENTIAL_MODE   (0)
#define STEERING_CENTER     (2535)
#define THROTTLE_FILTER_POLES (10)

#define CANTX               (3)
#define CANRX               (4)
#define THROTTLE_ID         (0x040)
#define STEERING_ID         (0x041)
#define LEFT_RPM_ID         (0x042)
#define RIGHT_RPM_ID        (0x043)

#define THROTTLE_MIN        (1350)
#define THROTTLE_MAX        (3960)
// buggyTest.h

#ifndef _BUGGYTEST_h
#define _BUGGYTEST_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

class BuggyTestClass
{
 protected:


 public:
	void init();
};

extern BuggyTestClass BuggyTest;

#endif


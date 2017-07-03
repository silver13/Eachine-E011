

#include <stdbool.h>
#include "pid.h"
#include "util.h"
#include "config.h"
#include "defines.h"

#define APIDNUMBER 2

// ANGLE PIDS - used in level mode
// acro pids are also used at the same time in level mode

// yaw is done by the rate yaw pid
// Kp                       ROLL     PITCH  
float apidkp[APIDNUMBER] = {14e-2, 14e-2 };

// Kd
float apidkd[APIDNUMBER] = { 10.0e-1, 10.0e-1 };

// code variables below

// rate limit
#define OUTLIMIT_FLOAT (float)LEVEL_MAX_RATE*DEGTORAD

#define ITERMLIMIT_FLOAT OUTLIMIT_FLOAT


extern int onground;
extern float looptime;
extern float gyro[3];

float apidoutput[APIDNUMBER];
float angleerror[3];


float lasterror[APIDNUMBER];

float apid(int x)
{

	// P term
	apidoutput[x] = angleerror[x] * apidkp[x];


extern float timefactor;
      
    apidoutput[x] = apidoutput[x] + (angleerror[x] - lasterror[x]) * apidkd[x] * timefactor;
    lasterror[x] = angleerror[x];

	limitf(&apidoutput[x], OUTLIMIT_FLOAT);


	return apidoutput[x];
}

// Defines the interfaces and function prototypes for the GNC processing stub.
// # () & !

#ifndef __INC_GNC_H_
#define __INC_GNC_H_

/* Constants for static allocation.  */
const unsigned int maxNumActuators = 12;
const unsigned int maxNumImu       =  4;
const unsigned int maxNumGnss      =  4;
const unsigned int maxNumStrTrk    =  4;

/* Modify this to change thread state. */
// bool threadRun = true;

/* GNC Configuration. */
typedef struct
{
    unsigned int numImu;
    unsigned int numGnss;
    unsigned int numStr;
} gncConfig_t;

/* Actuator State. */
typedef struct
{
    int actuatorState;
} actuatorData_t;

enum sensorIn
{
    IMU,
    GNSS,
    STK
};

/* Init Function Prototype */
int gncInit();

/* Step Function Prototype */
void gncStep();

/* Termintae Function Prototype */
int gncTerminate();

/* GNC Compute Output. */
int gncActuate(actuatorData_t* act);

#endif  // __INC_GNC_H_
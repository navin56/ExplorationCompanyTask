// Defines the interfaces and function prototypes for the GNC processing stub.
// # () & !

#ifndef __INC_GNC_H_
#define __INC_GNC_H_
#include <stdint.h>
#include "config.h"

/* Actuator State. */
typedef struct
{
    int actuatorState[6];
} actuatorData_t;

/* Init Function Prototype */
int gncInit();

/* Step Function Prototype */
void gncStep();

/* Termintae Function Prototype */
int gncTerminate();

/* GNC Compute Output. */
void gncActuate(sensorIn_e sensor, actuatorData_t* actDat);

#endif  // __INC_GNC_H_
// File contains all the IPC Address, Ports and Sensor Config.

// () #
#ifndef __LIBINC_CONFIG_H_
#define __LIBINC_CONFIG_H_

#include <stdint.h>
#include "imuInterface.h"
#include "gnssInterface.h"
#include "strInterface.h"

/* Constants for static allocation.  */
// const unsigned int maxNumActuators = 12;
// const unsigned int maxNumImu       =  4;
// const unsigned int maxNumGnss      =  4;
// const unsigned int maxNumStrTrk    =  4;

/* Why can't gcc treat const as a compile time const ? Why do i have to resort to pre-processor directives. Sigh. */
#define maxNumActuators 12U
#define maxNumImu        4U
#define maxNumGnss       4U
#define maxNumStrTrk     4U 
#define numGncSensorIf   3U

/* Number of inputs to GNC. */
// const unsigned int numGncSensorIf  =  3;

typedef enum 
{
    IMU  = 0,
    GNSS = 1,
    STK  = 2
} sensorIn_e;

/* Sensor Configuration. */
typedef struct
{
    gnssConfig_t   gnssConf;
    imuConfig_t    imuConf;
    strTrkConfig_t strConf;
} sensorConfig_t;

/* Constants for file Names. */
char imuFileName[]  = "../inputData/imuSens.npy";
char gnssFileName[] = "../inputData/gnssSens.npy";
char strFileName[]  = "../inputData/strSens.npy";

const char inFp[3][30] = {
                        "../inputData/imuSens.npy",
                        "../inputData/gnssSens.npy",
                        "../inputData/strSens.npy"
                    };

/* IPC Address and Port Definitions. */
const char      IPCAddr[]      = "127.0.0.1";
const uint16_t  ImuIpcPort     = 60010;
const uint16_t  GnssIpcPort    = 60020;
const uint16_t  StrIpcPort     = 60030;
const uint16_t  ActIpcPort     = 60000;

const uint16_t  imuFdirPort    = 50010;
const uint16_t  gnssFdirPort   = 50020;
const uint16_t  strFdirPort    = 50030;

/* Utility Functions. */
void setNumSensors(sensorConfig_t* sCfg, unsigned int numImu, unsigned int numGnss, unsigned int numStrTrk)
{
    if (numImu <= maxNumImu)
    {
        sCfg->imuConf.numImuSensors = numImu;
    }
    else
    {
        sCfg->imuConf.numImuSensors = 0;
    }
    
    if (numGnss <= maxNumGnss)
    {
        sCfg->gnssConf.numGnssSensors = numGnss;
    }
    else
    {
        sCfg->gnssConf.numGnssSensors = 0;
    }
    
    if (numStrTrk <= maxNumStrTrk)
    {
        sCfg->strConf.numStrTrk = numStrTrk;
    }
    else
    {
        sCfg->strConf.numStrTrk = 0;
    }
}

void setSensorLatency(sensorConfig_t* sCfg, double imuMs, double gnssMs, double strMs)
{
    sCfg->imuConf.latency_ms  = imuMs;
    sCfg->gnssConf.latency_ms = gnssMs;
    sCfg->strConf.latency_ms  = strMs;
}

#endif  // 

// Implements Sensor FDIR Handling.

#include "config.h"
#include "interfaceLib.h"

/* Static Memory Allocations. */
ipcConfig_t imuMsgConf[maxNumImu];
ipcConfig_t gnssMsgConf[maxNumGnss];
ipcConfig_t strMsgConf[maxNumStrTrk];

ipcConfig_t gncSendIpc[numGncSensorIf];

/* Allocate buffer and Sensor data structures. */
imuData_u    imuMsg[maxNumImu];
gnssData_u   gnssMsg[maxNumGnss];
strTrkData_u strMsg[maxNumStrTrk];

typedef struct
{
    ipcConfig_t* inputCfg;
    sensorIn_e   sensor;
    unsigned int numSensors;
    ipcConfig_t* outputCfg;
} taskArg_t;


void initFdirReadIpc(ipcConfig_t* cfg, uint16_t numSensors, uint16_t basePort);

void initGncSendIpc(ipcConfig_t* cfg, unsigned int numIf);

void fdirThread(taskArg_t* args);


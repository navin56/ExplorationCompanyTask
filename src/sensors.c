// Implements reading Sensor data and sending message on the network.

#include <string.h>

#include "threadLib.h"
#include "interfaceLib.h"
#include "config.h"

uint8_t imuMsgBuf[sizeof(imuData_t)];
uint8_t gnssMsgBuf[sizeof(gnssData_t)];
uint8_t strMsgBuf[sizeof(strTrkData_t)];

task_t  imuTask;
task_t  gnssTask;
task_t  strTask;

uint8_t fdir = 0;

/* Iterations after which a sensor Fault occurs. */
const uint8_t fdirEnableIter = 10;

typedef struct
{
    ipcConfig_t*    cfg;
    unsigned int    numSensors;
    uint8_t*        dataBuf;
    task_t*         tCfg;
    npy_array_t*    np;
} taskArg_t;

/* Function to read IMU data from numpy binary file. */
void* getImuDataNpy(void* argP)
{
    taskArg_t* arg = (taskArg_t* ) argP;
    double temp;
    char * ptr;
    size_t iter = arg->np->shape[0];
    size_t idx = 0;
    ssize_t retval;
    imuData_t rawData;
    rawData.tInc = 0.01;
    for (size_t i = 0; i < iter; i++)
    {
        ptr = arg->np->data;
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.velInc[0] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.velInc[1] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.velInc[2] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.angInc[0] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.angInc[1] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.angInc[2] = temp;
        idx += sizeof(double);             

        memcpy(arg->dataBuf, (void*) &rawData, sizeof(rawData));
        if ((fdir == 1) && (i > fdirEnableIter))
        {
            /* Reduce number of working sensors to 2. */
            arg->numSensors = 2;
            printf("Fault Injected for IMU. \n");
            /* Just so we do not enter this peice of code again. */
            fdir = 0;
        }

        /* Sleep. */
        for (size_t i = 0; i < arg->numSensors; i++)
        {
            retval = sendMsgIPC(&arg->cfg[i], arg->dataBuf, sizeof(rawData));
        }
        printf("Sent %d IMU Msg.  %ld \n", arg->numSensors, retval);
        retval = threadSleep(arg->tCfg);
    }
    return NULL;
}
/* Function to read GNSS data from numpy binary file. */
void* getGnssDataNpy(void* argP)
{
    taskArg_t* arg = (taskArg_t* ) argP;
    double temp;
    char * ptr;
    size_t nRows = arg->np->shape[0];
    size_t idx = 0;
    ssize_t retval;
    gnssData_t rawData;
    rawData.DOP = 0.8;
    rawData.validity = 1;
    for (size_t i = 0; i < nRows; i++)
    {
        ptr = arg->np->data;
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.positionGd_m[0] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.positionGd_m[1] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.positionGd_m[2] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.velocityEnu_m_s[0] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.velocityEnu_m_s[1] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.velocityEnu_m_s[2] = temp;
        idx += sizeof(double);             

        memcpy(arg->dataBuf, (void*) &rawData, sizeof(rawData));
        /* Sleep. */
        for (size_t i = 0; i < arg->numSensors; i++)
        {
            retval = sendMsgIPC(&arg->cfg[i], arg->dataBuf, sizeof(rawData));
        }
        printf("Sent %d GNSS Msg. %ld \n", arg->numSensors, retval);
        retval = threadSleep(arg->tCfg);
    }
    return NULL;
}
/* Function to read Star Tracker data from numpy binary file. */
void* getStrDataNpy(void* argP)
{
    taskArg_t* arg = (taskArg_t* ) argP;
    double temp;
    char * ptr;
    size_t nRows = arg->np->shape[0];
    size_t idx = 0;
    ssize_t retval;
    strTrkData_t rawData;

    for (size_t i = 0; i < nRows; i++)
    {
        ptr = arg->np->data;
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.timeTag = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.quaternion[0] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.quaternion[1] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.quaternion[2] = temp;
        idx += sizeof(double);
        memcpy(&temp, ptr + idx, sizeof(double));
        rawData.quaternion[3] = temp;

        memcpy(arg->dataBuf, (void* ) &rawData, sizeof(rawData));

        for (size_t i = 0; i < arg->numSensors; i++)
        {
            retval = sendMsgIPC(&arg->cfg[i], strMsgBuf, sizeof(rawData));
        }
        printf("Sent %d Star Tracker Msg %ld \n", arg->numSensors, retval);
        retval = threadSleep(arg->tCfg);
    }
    return NULL;
}

int main(int argc, char* argv[])
{
    /* Unused parameter. */
    (void) argv;

    /* FDIR Enable */
    if (argc > 1)
    {
        fdir = 1;
    }

    /* IPC Config. */
    ipcConfig_t imuMsgConf[maxNumImu];
    ipcConfig_t gnssMsgConf[maxNumGnss];
    ipcConfig_t strMsgConf[maxNumStrTrk];

    /* Task Arguments Config. */
    taskArg_t args[numGncSensorIf];

    /* DataBuffer. */
    imuData_u    imuBuf;
    gnssData_u   gnssBuf;
    strTrkData_u stkBuf;

    args[0].dataBuf = imuBuf.dataBuf;
    args[1].dataBuf = gnssBuf.dataBuf;
    args[2].dataBuf = stkBuf.dataBuf;

    /* File Interface Configs. */
    interfaceCfg_t inputIf[numGncSensorIf];

    /* Sensor Config. */
    sensorConfig_t sensConf;

    /* Task Property */
    task_t task[numGncSensorIf];
 
    for (size_t i = 0; i < numGncSensorIf; i++)
    {
        args[i].tCfg = &task[i];
    }

    if (fdir == 1)
    {
        setNumSensors(&sensConf, 3, 3, 3);        
    }
    else
    {
        setNumSensors(&sensConf, 1, 1, 1);
    }

    args[0].numSensors = sensConf.imuConf.numImuSensors;
    args[1].numSensors = sensConf.gnssConf.numGnssSensors;
    args[2].numSensors = sensConf.strConf.numStrTrk;

    /* Init File interfaces. */
    for (size_t i = 0; i < numGncSensorIf; i++)
    {
        setInterface(&inputIf[i], INPUT, (char *) inFp[i]);
        args[i].np = npyLoadData(&inputIf[i]);
    }

    /* Set up Sockets. */
    for (size_t i = 0; i < args[0].numSensors; i++)
    {
        if (fdir == 1)
        {
            /* Set the fdir port. */
            setIpcAddrPort(&imuMsgConf[i], (char* ) IPCAddr, imuFdirPort + i, OUTPUT);
        }
        else
        {
            /* Set the gnc port. */
            setIpcAddrPort(&imuMsgConf[i], (char* ) IPCAddr, ImuIpcPort + i, OUTPUT);
        }
    }
    args[0].cfg = imuMsgConf;

    for (size_t i = 0; i < args[1].numSensors; i++)
    {
        if (fdir == 1)
        {
            setIpcAddrPort(&gnssMsgConf[i], (char* ) IPCAddr, gnssFdirPort + i, OUTPUT);    
        }
        else
        {
            setIpcAddrPort(&gnssMsgConf[i], (char* ) IPCAddr, GnssIpcPort + i, OUTPUT);
        }
    }
    args[1].cfg = gnssMsgConf;

    for (size_t i = 0; i < args[2].numSensors; i++)
    {
        if (fdir == 1)
        {
            setIpcAddrPort(&strMsgConf[i], (char* ) IPCAddr, strFdirPort + i, OUTPUT);
        }
        else
        {
            setIpcAddrPort(&strMsgConf[i], (char* ) IPCAddr, StrIpcPort + i, OUTPUT);
        }
    }
    args[2].cfg = strMsgConf;

    setTaskPeriod(args[0].tCfg, 1.0);
    setTaskPeriod(args[1].tCfg, 0.5);
    setTaskPeriod(args[2].tCfg, 0.1);

    pthread_create(&imuTask.taskThread, NULL,  &getImuDataNpy, (void* ) &args[0]);
    pthread_create(&gnssTask.taskThread, NULL, &getGnssDataNpy, (void* ) &args[1]);
    pthread_create(&strTask.taskThread, NULL,  &getStrDataNpy, (void* ) &args[2]);
    pthread_join(imuTask.taskThread, NULL);
    pthread_join(gnssTask.taskThread, NULL);
    pthread_join(strTask.taskThread, NULL);
}

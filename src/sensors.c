// Implements reading Sensor data and sending message on the network.

#include <string.h>

#include "threadLib.h"
#include "interfaceLib.h"
#include "imuInterface.h"
#include "gnssInterface.h"
#include "strInterface.h"

npy_array_t* imuData;
npy_array_t* gnssData;
npy_array_t* strData;

uint8_t imuMsgBuf[sizeof(imuData_t)];
uint8_t gnssMsgBuf[sizeof(gnssData_t)];
uint8_t strMsgBuf[sizeof(strTrkData_t)];

task_t  imuTask;
task_t  gnssTask;
task_t  strTask;

void getImuData()
{
    imuData_t rawData;
    rawData.tInc      = 0.01;
    rawData.validity  = 1;
    
    rawData.angInc[0] = 0.05;
    rawData.angInc[1] = -0.15;
    rawData.angInc[2] = 2.0;

    rawData.velInc[0] = 2.5;
    rawData.velInc[1] = 1.05;
    rawData.velInc[2] = -8.75;

    memcpy(imuMsgBuf, (void*) &rawData, sizeof(rawData));
}

void getImuDataNpy(ipcConfig_t* imuIpc)
{
    double temp;
    char * ptr;
    size_t iter = imuData->shape[0];
    size_t idx = 0;
    ssize_t retval;
    imuData_t rawData;
    rawData.tInc = 0.01;
    for (size_t i = 0; i < iter; i++)
    {
        ptr = imuData->data;
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

        memcpy(imuMsgBuf, (void*) &rawData, sizeof(rawData));
        /* Sleep. */
        retval = sendMsgIPC(imuIpc, imuMsgBuf, sizeof(imuMsgBuf));
        printf("Sent IMU Msg.  %ld \n", retval);
        retval = threadSleep(&imuTask);
    }
}

void getGnssData()
{
    gnssData_t rawData;
    
    rawData.DOP = 0.8;
    
    rawData.positionGd_m[0] = 0.69;
    rawData.positionGd_m[1] = 0.45;
    rawData.positionGd_m[2] = 6.05;

    rawData.velocityEnu_m_s[0] = 0.0;
    rawData.velocityEnu_m_s[1] = 2.5;
    rawData.velocityEnu_m_s[2] = 0.0;

    rawData.validity = 0;
    memcpy(gnssMsgBuf, (void*) &rawData, sizeof(rawData));
}

void getGnssDataNpy(ipcConfig_t* gnssIpc)
{
    double temp;
    char * ptr;
    size_t nRows = gnssData->shape[0];
    size_t idx = 0;
    ssize_t retval;
    gnssData_t rawData;
    rawData.DOP = 0.8;
    rawData.validity = 1;
    for (size_t i = 0; i < nRows; i++)
    {
        ptr = gnssData->data;
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

        memcpy(gnssMsgBuf, (void*) &rawData, sizeof(rawData));
        /* Sleep. */
        retval = sendMsgIPC(gnssIpc, gnssMsgBuf, sizeof(gnssMsgBuf));
        printf("Sent GNSS Msg.  %ld \n", retval);
        retval = threadSleep(&gnssTask);
    }
}

void sendImuData(ipcConfig_t* imuIpc)
{
    while (1)
    {
        getImuData();
        ssize_t retval;
    }
}

void sendGnssData(ipcConfig_t* gnssIpc)
{
    while (1)
    {
        getGnssData();
        ssize_t retval;
        retval = sendMsgIPC(gnssIpc, gnssMsgBuf, sizeof(gnssMsgBuf));
        printf("Sent GNSS Msg. %ld \n", retval);
        retval = threadSleep(&gnssTask);
    }
}

void getStrDataNpy(ipcConfig_t* strIpc)
{
    double temp;
    char * ptr;
    size_t nRows = strData->shape[0];
    size_t idx = 0;
    ssize_t retval;
    strTrkData_t rawData;

    for (size_t i = 0; i < nRows; i++)
    {
        ptr = strData->data;
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

        memcpy(strMsgBuf, (void* ) &rawData, sizeof(strMsgBuf));
        retval = sendMsgIPC(strIpc, strMsgBuf, sizeof(strMsgBuf));
        printf("Sent Star Tracker Msg \n");
        retval = threadSleep(&strTask);
    }

}

int main()
{
    ipcConfig_t imuIpc;
    ipcConfig_t gnssIpc;
    ipcConfig_t strIpc;

    interfaceCfg_t imuFile;
    interfaceCfg_t gnssFile;
    interfaceCfg_t strFile;

    char imuFileName[]  = "../inputData/imuSens.npy";
    char gnssFileName[] = "../inputData/gnssSens.npy";
    char strFileName[]  = "../inputData/strSens.npy";

    imuFile.direction = INPUT;
    imuFile.filePath  = imuFileName;

    gnssFile.filePath  = gnssFileName;
    gnssFile.direction = INPUT;

    strFile.filePath   = strFileName;
    strFile.direction  = INPUT;

    initInterface(&imuFile);
    imuData = npyLoadData(&imuFile);

    initInterface(&gnssFile);
    gnssData = npyLoadData(&gnssFile);

    initInterface(&strFile);
    strData  = npyLoadData(&strFile);

    char ipAddr[]     = "127.0.0.1";
    uint16_t imuPort  = 60010;
    uint16_t gnssPort = 60020;
    uint16_t strPort  = 60030;

    imuIpc.ipAddress    = ipAddr;
    imuIpc.port         = imuPort;
    imuIpc.direction    = OUTPUT;

    gnssIpc.ipAddress   = ipAddr;
    gnssIpc.port        = gnssPort;
    gnssIpc.direction   = OUTPUT;

    strIpc.ipAddress    = ipAddr;
    strIpc.port         = strPort;
    strIpc.direction    = OUTPUT;

    setTaskPeriod(&imuTask, 10.0);
    setTaskPeriod(&gnssTask, 1.0);
    setTaskPeriod(&strTask, 0.1);


    imuIpc.ipcSock  = initIPC(&imuIpc);
    gnssIpc.ipcSock = initIPC(&gnssIpc);
    strIpc.ipcSock  = initIPC(&strIpc);


    pthread_create(&imuTask.taskThread, NULL, &getImuDataNpy, &imuIpc);
    pthread_create(&gnssTask.taskThread, NULL, &getGnssDataNpy, &gnssIpc);
    pthread_create(&strTask.taskThread, NULL, &getStrDataNpy, &strIpc);
    pthread_join(imuTask.taskThread, NULL);
    pthread_join(gnssTask.taskThread, NULL);
    pthread_join(strTask.taskThread, NULL);
}

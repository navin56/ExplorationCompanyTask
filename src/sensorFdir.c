// ()

#include "sensorFdir.h"
#include "threadLib.h"

void initFdirReadIpc(ipcConfig_t* cfg, uint16_t numSensors, uint16_t basePort)
{
    for (size_t i = 0; i < numSensors; i++)
    {
        /* Application receives sensor data. */
        cfg[i].direction = INPUT;
        cfg[i].ipAddress = (char *) IPCAddr;
        cfg[i].port      = basePort + i;
        initIPC(&cfg[i]);
        /* Set up the Poll FD. */
        cfg[i].sockPoll.fd     = cfg[i].ipcSock;
        cfg[i].sockPoll.events = POLLIN;
    }
}

void initGncSendIpc(ipcConfig_t* cfg, unsigned int numIf)
{
    /* Initialize the sockets to transmit data. */
    for (size_t i = 0; i < numIf; i++)
    {
        cfg[i].direction = OUTPUT;
        cfg[i].ipAddress = (char* ) IPCAddr;
    }
    /* Set the ports Individually. */
    cfg[IMU].port  = ImuIpcPort;
    cfg[GNSS].port = GnssIpcPort;
    cfg[STK].port  = StrIpcPort;

    /* Init the Sockets. */
    for (size_t i = 0; i < numIf; i++)
    {
        initIPC(&cfg[i]);
    }
}

unsigned int fdirSelect(taskArg_t* args, unsigned int numRx)
{
    if (numRx == args->numSensors)
    {
        /* All Sensors Working, select index 0 */
        return 0;
    }
    else if (numRx < args->numSensors)
    {
        /* We have a problem now. */
        /* Select index less than numSensors. */
        args->numSensors = numRx;
        return numRx;
    }
}

void* fdirThread(void* argP)
{
    taskArg_t* args = (taskArg_t *) argP;
    ssize_t ret;
    while (1)
    {
        /* Receive IMU Data. */
        for (size_t i = 0; i < args->numSensors; i++)
        {
            switch (args->sensor)
            {
            case IMU:
                ret = recvMsgIPC(&args->inputCfg[i], imuMsg[i].dataBuf, sizeof(imuData_t));
                rxImu++;
                break;

            case GNSS:
                ret = recvMsgIPC(&args->inputCfg[i], gnssMsg[i].dataBuf, sizeof(gnssData_t));
                rxGnss++;
                break;

            case STK:
                ret = recvMsgIPC(&args->inputCfg[i], strMsg[i].dataBuf, sizeof(strTrkData_t));
                rxStr++;
                break;
            
            default:
                break;
            }
        }
        (void) ret;
        /* Select One sensor and Transmit. Hardcoded for now. */
        switch (args->sensor)
        {
            unsigned int index;
            case IMU:
                index = fdirSelect(args, rxImu);
                printf("Rx %d IMU Packets, Selecting IMU %d \n", rxImu, index);
                sendMsgIPC(&args->outputCfg[args->sensor], imuMsg[index].dataBuf, sizeof(imuData_t));
                /* Reset the receive counter. */
                rxImu = 0;
                break;

            case GNSS:
                index = fdirSelect(args, rxGnss);
                printf("Rx %d GNSS Packets, Selecting GNSS %d \n", rxGnss, index);
                sendMsgIPC(&args->outputCfg[args->sensor], gnssMsg[index].dataBuf, sizeof(gnssData_t));
                rxGnss = 0;
                break;

            case STK:
                index = fdirSelect(args, rxStr);
                printf("Rx %d STR Packets, Selecting STR %d \n", rxStr, index);
                sendMsgIPC(&args->outputCfg[args->sensor], strMsg[index].dataBuf, sizeof(strTrkData_t));
                rxStr = 0;
                break;

            default:
                break;
        }
    }
    return NULL;
}

int main()
{
    sensorConfig_t cfg;
    task_t         fdirTasks[numGncSensorIf];
    taskArg_t      arg[3];

    /* Classic TMR. */
    setNumSensors(&cfg, 3, 3, 3);
    
    /* Initialize the sockets. */
    initFdirReadIpc(imuMsgConf, cfg.imuConf.numImuSensors, imuFdirPort);
    initFdirReadIpc(gnssMsgConf, cfg.imuConf.numImuSensors, gnssFdirPort);
    initFdirReadIpc(strMsgConf, cfg.imuConf.numImuSensors, strFdirPort);

    /* Init the output Sockets. */
    initGncSendIpc(gncSendIpc, numGncSensorIf);
    
    /* Set up argument struct. */
    arg[0].inputCfg    = imuMsgConf;
    arg[0].sensor      = IMU;
    arg[0].numSensors  = cfg.imuConf.numImuSensors;
    arg[0].outputCfg   = &gncSendIpc[0];

    arg[1].inputCfg    = gnssMsgConf;
    arg[1].sensor      = GNSS;
    arg[1].numSensors  = cfg.gnssConf.numGnssSensors;
    arg[1].outputCfg   = &gncSendIpc[1];

    arg[2].inputCfg    = strMsgConf;
    arg[2].sensor      = STK;
    arg[2].numSensors  = cfg.strConf.numStrTrk;
    arg[2].outputCfg   = &gncSendIpc[2];

    /* Start the Threads. */
    for (size_t i = 0; i < numGncSensorIf; i++)
    {
        pthread_create(&fdirTasks[i].taskThread, NULL, fdirThread, (void *) &arg[i]);
    }

    pthread_join(fdirTasks[0].taskThread, NULL);
    pthread_join(fdirTasks[1].taskThread, NULL);
    pthread_join(fdirTasks[2].taskThread, NULL);
    return 0;
}

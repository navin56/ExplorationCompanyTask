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

void fdirThread(taskArg_t* args)
{   ssize_t ret;
    while (1)
    {
        /* Receive IMU Data. */
        for (size_t i = 0; i < args->numSensors; i++)
        {
            switch (args->sensor)
            {
            case IMU:
                ret = recvMsgIPC(&args->inputCfg[i], imuMsg[i].dataBuf, sizeof(imuData_t));
                printf("Received IMU Data : %ld \n", ret);
                break;

            case GNSS:
                ret = recvMsgIPC(&args->inputCfg[i], gnssMsg[i].dataBuf, sizeof(gnssData_t));
                printf("Received GNSS Data : %ld \n", ret);
                break;

            case STK:
                ret = recvMsgIPC(&args->inputCfg[i], strMsg[i].dataBuf, sizeof(strTrkData_t));
                printf("Received STK Data : %ld \n", ret);
                break;
            
            default:
                break;
            }
        }
        /* Select One sensor and Transmit. Hardcoded for now. */
        switch (args->sensor)
        {
        case IMU:
            sendMsgIPC(&args->outputCfg[args->sensor], imuMsg[1].dataBuf, sizeof(imuData_t));
            break;
        
        case GNSS:
            sendMsgIPC(&args->outputCfg[args->sensor], gnssMsg[1].dataBuf, sizeof(gnssData_t));
            break;

        case STK:
            sendMsgIPC(&args->outputCfg[args->sensor], strMsg[1].dataBuf, sizeof(strTrkData_t));
            break;

        default:
            break;
        }
    }
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

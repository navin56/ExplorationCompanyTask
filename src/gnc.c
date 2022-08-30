// & ()

#include <stdio.h>
#include "gnc.h"
#include "threadLib.h"
#include "interfaceLib.h"

struct pollfd fds[3];

/* IPC structures. */
ipcConfig_t imuMsgConf;
ipcConfig_t gnssMsgConf;
ipcConfig_t strMsgConf;

imuData_u    imuMsg;
gnssData_u   gnssMsg;
strTrkData_u stkMsg;

int gncInit()
{
    printf("GNC Init... \n");
    /* Set Socket IP and Ports. */
    setIpcAddrPort(&imuMsgConf,  (char *) IPCAddr, ImuIpcPort, INPUT);
    setIpcAddrPort(&gnssMsgConf, (char *) IPCAddr, GnssIpcPort, INPUT);
    setIpcAddrPort(&strMsgConf,  (char *) IPCAddr, StrIpcPort, INPUT);
    /* Set socket Directions. */
    imuMsgConf.direction  = INPUT;
    gnssMsgConf.direction = INPUT;
    strMsgConf.direction  = INPUT;
    /* Init the sockets. */
    initIPC(&imuMsgConf);
    initIPC(&gnssMsgConf);
    initIPC(&strMsgConf);
    /* Set sockets to the poll struct File descriptor. */
    fds[0].fd = imuMsgConf.ipcSock;
    fds[1].fd = gnssMsgConf.ipcSock;
    fds[2].fd = strMsgConf.ipcSock;

    initPollFd(fds, 3, POLLIN);
    return 0;
}

/* GNC Actuate. */
void gncActuate(sensorIn_e sensor, actuatorData_t* actDat)
{
    switch (sensor)
    {
        case IMU:
            recvMsgIPC(&imuMsgConf, imuMsg.dataBuf, sizeof(imuData_t));
            printf("Setting Actuators {5} to On \n");
            break;

        case GNSS:
            recvMsgIPC(&gnssMsgConf, gnssMsg.dataBuf, sizeof(gnssData_t));
            printf("Setting Actuators {2, 6} to On \n");
            break;

        case STK:
            recvMsgIPC(&strMsgConf, stkMsg.dataBuf, sizeof(strTrkData_t));
            printf("Settings Actuators {1, 2, 3} to On \n");
            break;

        default:
            break;
    }
}

int main()
{
    task_t      gncTask;

    gncInit();
    setTaskPeriod(&gncTask, 10.0);

    unsigned int timeOutCtr = 0;

    while (1)
    {
        int ret;
        ret = poll(fds, 3, 1000);
        /* Positive Retval indicates success. */
        if (ret > 0)
        {
            for (size_t i = 0; i < 3; i++)
            {
                /* Find the FD that caused poll to return. */
                if (fds[i].revents == fds[i].events)
                {
                    /* Actuate away. */
                    gncActuate((sensorIn_e) i, NULL);
                }
            }
        }
        else if(ret == 0)
        {
            /* No data to be read. Or socket timeout. */
            printf("Poll Timed out. \n");
            timeOutCtr++;
        }
        else
        {
            /* Error Handling. */
            perror("Poll Error. \n");
        }
        printf("Timeout: %d \n", timeOutCtr);
    }
    return 0;
}

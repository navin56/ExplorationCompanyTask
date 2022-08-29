// & ()

#include <stdio.h>
#include "gnc.h"
#include "threadLib.h"
#include "interfaceLib.h"
#include "imuInterface.h"

const char imuMsgIp[]      = "127.0.0.1";
const uint16_t imuMsgPort  = 60010;

int gncInit(task_t* taskInfo)
{
    printf("GNC Init... \n");
    return 0;
}

void gncStep(task_t* taskInfo)
{
    ipcConfig_t imuMsgConf;
    imuMsgConf.ipAddress = imuMsgIp;
    imuMsgConf.port      = imuMsgPort;
    imuMsgConf.direction = INPUT;
    imuMsgConf.ipcSock   = initIPC(&imuMsgConf);

    while(1)
    {
        /* Hey There. */
        imuData_t imuData;
        uint8_t   recvBuf[sizeof(imuData)];
        ssize_t   retval;

        retval = recvMsgIPC(&imuMsgConf, (void* )&imuData, sizeof(recvBuf));

    }
}

int main()
{
    task_t      gncTask;

    gncInit(&gncTask);
    setTaskPeriod(&gncTask, 10.0);

    pthread_create(&gncTask.taskThread, NULL, &gncStep, &gncTask);
    pthread_join(gncTask.taskThread, NULL);

    return 0;
}
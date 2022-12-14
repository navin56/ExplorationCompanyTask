// 
#include "interfaceLib.h"

int initInterface(interfaceCfg_t* cfg)
{
    if (cfg->direction == INPUT)
    {
        cfg->interfaceFp = fopen(cfg->filePath, "r");
    }
    else if (cfg->direction == OUTPUT)
    {
        cfg->interfaceFp = fopen(cfg->filePath, "w");
    }
    else
    {
        /* Incorrect direction passed. */
        return -1;
    }
    if (cfg->interfaceFp == NULL)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

int closeInterface(interfaceCfg_t* cfg)
{
    return fclose(cfg->interfaceFp);
}

npy_array_t* npyLoadData(interfaceCfg_t* cfg)
{
    return npy_array_load(cfg->filePath);
}

void setInterface(interfaceCfg_t* cfg, enum interfaceType type, char* filename)
{
    cfg->direction = type;
    cfg->filePath  = filename;
    initInterface(cfg);
}

int initIPC(ipcConfig_t* cfg)
{
    int sock = -1;

    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if(sock == -1)
    {
        /* Error */
        perror("Socket Creation Failed.");
        return -1;
    }

    cfg->si.sin_family      = AF_INET;
    cfg->si.sin_port        = htons(cfg->port);
    cfg->si.sin_addr.s_addr = inet_addr(cfg->ipAddress);

    if (cfg->direction == INPUT)
    {
        /* Reuse ports */
        int yes = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0)
        {
            perror("Error Seting Socket Options.");
        }
        

        if (bind(sock, (struct sockaddr *) &cfg->si, sizeof(cfg->si)) == -1)
        {
            perror("Bind Failed.");
            return -1;
        }
    }
    /* Set the socket in the structure itself. */
    cfg->ipcSock = sock;
    /* Code is redundant. */
    return sock;
}

ssize_t sendMsgIPC(ipcConfig_t* cfg, uint8_t* dataBuf, size_t dataBufSize)
{
    ssize_t retVal;
    retVal = sendto(cfg->ipcSock, dataBuf, dataBufSize, 0, (struct sockaddr *) &cfg->si, sizeof(cfg->si));
    return retVal;
}

ssize_t recvMsgIPC(ipcConfig_t* cfg, uint8_t* dataBuf, size_t dataBufSize)
{
    ssize_t retVal;
    socklen_t addrSize;
    addrSize = sizeof(cfg->si);
    retVal = recvfrom(cfg->ipcSock, dataBuf, dataBufSize, 0, (struct sockaddr *) &cfg->si, &addrSize);
    return retVal;
}

void initPollFd(struct pollfd* fds, unsigned int numFd, int event)
{
    for (size_t i = 0; i < numFd; i++)
    {
        fds[i].events = event;
    }
}

void setIpcAddrPort(ipcConfig_t* cfg, char* addr, uint16_t port, enum interfaceType type)
{
    cfg->ipAddress = addr;
    cfg->port      = port;
    cfg->direction = type;
    initIPC(cfg);
}

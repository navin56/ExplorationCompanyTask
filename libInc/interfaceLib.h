// This file implements a mock serial interface by reading from a file.
// Upper layers of the software shall use functions defined here.
// UDP trasnmit and receive are also implemented here for IPC.

#ifndef __LIBINC_INTERFACELIB_H_
#define __LIBINC_INTERFACELIB_H_

#include <stdint.h>
#include <stdio.h>               // File and IO Operations.

#include <sys/types.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include "npy_array.h"

enum interfaceType
{
    INPUT  = 42,
    OUTPUT = 100
};

typedef struct
{
    char*              filePath;                    //< Store Interface Path.
    enum interfaceType direction;                   //< Enum to store if the interface is an input or output.
    FILE*              interfaceFp;                 //< File pointer.
    double             interfaceLatency;            //< Adjustable Interface Latency.
    size_t             expectedReadSize;            //< Nominal Read Size in bytes. Corresponds to packet size from sensor.
    char*              fmtSpec;                     //< Specify format for fscanf to parse csv. Device specifc driver to provide this.
}interfaceCfg_t;

typedef struct
{
    char*              ipAddress;
    uint16_t           port;
    int                ipcSock;
    struct sockaddr_in si;
    enum interfaceType direction;
} ipcConfig_t;

int initInterface(interfaceCfg_t* cfg);

npy_array_t* npyLoadData(interfaceCfg_t* cfg);

int closeInterface(interfaceCfg_t* cfg);

int initIPC(ipcConfig_t* cfg);

ssize_t sendMsgIPC(ipcConfig_t* cfg, uint8_t* dataBuf, size_t dataBufSize);

ssize_t recvMsgIPC(ipcConfig_t* cfg, uint8_t* dataBuf, size_t dataBufSize);

#endif  // __LIBINC_INTERFACELIB_H_
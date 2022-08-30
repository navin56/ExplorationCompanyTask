// Interface Details for IMU Sensor to GNC.

typedef struct
{
    double velInc[3];
    double angInc[3];
    double tInc;
    int    validity;
} imuData_t;

typedef union
{
    imuData_t data;
    uint8_t   dataBuf[sizeof(imuData_t)];
} imuData_u;

typedef struct
{
    unsigned int numImuSensors;
    double       latency_ms;
    double       samplingRate;
} imuConfig_t;

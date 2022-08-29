// Interface Details for IMU Sensor to GNC.

typedef struct
{
    double velInc[3];
    double angInc[3];
    double tInc;
    int    validity;
} imuData_t;

typedef struct
{
    unsigned int numImuSensors;
    double       samplingRate;
} imuConfig_t;

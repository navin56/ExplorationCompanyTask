// Interface details for GNSS Sensor to GNC.

typedef struct
{
    double positionGd_m[3];
    double velocityEnu_m_s[3];
    double DOP;
    int    validity;
}gnssData_t;

typedef union gnssInterface
{
    gnssData_t data;
    uint8_t    dataBuf[sizeof(gnssData_t)];
} gnssData_u;


typedef struct
{
    unsigned int numGnssSensors;
    double       latency_ms;
    double       samplingFreq;
}gnssConfig_t;

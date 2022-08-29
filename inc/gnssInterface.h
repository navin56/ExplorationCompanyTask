// Interface details for GNSS Sensor to GNC.

typedef struct
{
    double positionGd_m[3];
    double velocityEnu_m_s[3];
    double DOP;
    int    validity;
}gnssData_t;

typedef struct
{
    unsigned int numGnssSensors;
    double       samplingFreq;
}gnssConfig_t;

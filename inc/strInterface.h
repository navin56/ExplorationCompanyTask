// Interface Details for Star Tracker to GNC.

typedef struct
{
    double timeTag;
    double quaternion[4];
} strTrkData_t;

typedef union
{
    strTrkData_t data;
    uint8_t      dataBuf[sizeof(strTrkData_t)];
} strTrkData_u;


typedef struct
{
    unsigned int numStrTrk;
    double       latency_ms;
    double       samplingFreq;
} strTrkConfig_t;

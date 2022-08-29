// Interface Details for Star Tracker to GNC.

typedef struct
{
    double timeTag;
    double quaternion[4];
} strTrkData_t;

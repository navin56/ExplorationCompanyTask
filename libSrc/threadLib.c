//
#include "threadLib.h"

void setTaskPeriod(task_t *taskInfo, double freq)
{
    /* Obtain time period in seconds. */
    double temp  = 1/freq;
    /* Fractional part of the double. */
    double frac  = temp - ((long) temp);
    
    long   seconds = (long) temp;
    long   ns      = (long) (frac * 1000000000);
    
    taskInfo->taskPeriod.tv_sec  = seconds;
    taskInfo->taskPeriod.tv_nsec = ns;
}

int threadSleep(task_t *taskInfo)
{
    return (nanosleep(&taskInfo->taskPeriod, NULL));
}

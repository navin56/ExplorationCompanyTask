// Small utility functions related to pthread
#ifndef __LIBINC_THREADLIB_H_
#define __LIBINC_THREADLIB_H_

#include <pthread.h>
#include <time.h>           // nanosleep Function

typedef struct
{
    pthread_t        taskThread;
    struct timespec  taskPeriod;
}task_t;

/* Utility Function to set task period. */
void setTaskPeriod(task_t *taskInfo, double freq);

/* Utility Function to sleep. */
int threadSleep(task_t *taskInfo);

#endif  // __LIBINC_THREADLIB_H_
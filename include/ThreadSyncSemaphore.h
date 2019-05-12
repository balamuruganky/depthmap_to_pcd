#ifndef SEMAPHORE_H
#define SEMAPHORE_H

#include "Common.h"

#include <semaphore.h>
#include <time.h>

class ThreadSyncSemaphore
{
public:
  ThreadSyncSemaphore ()
  {
    sem_init (&_ThreadSyncSemaphoreHandle, 1, 0);
  }
   ~ThreadSyncSemaphore ()
  {
    sem_close (&_ThreadSyncSemaphoreHandle);
    sem_destroy (&_ThreadSyncSemaphoreHandle);
  }

  int32_t SemaphorePost ()
  {
    return (sem_post (&_ThreadSyncSemaphoreHandle));
  }

  int32_t SemaphoreTimedWait (uint16_t TimeOutInSeconds = 5)
  {
    struct timespec ts;
    if (clock_gettime (CLOCK_REALTIME, &ts) == ERROR)
      {
	return ERROR;
      }
    ts.tv_sec += TimeOutInSeconds;

    return (sem_timedwait (&_ThreadSyncSemaphoreHandle, &ts));
  }

private:
  sem_t _ThreadSyncSemaphoreHandle;
};

#endif // SEMAPHORE_H

#ifndef _CONDITIONS_H
#define _CONDITIONS_H

#include <pthread.h>

class Condition
{
public:
  Condition ()
  {
    pthread_cond_init (&_ConditionVar, NULL);
  }

  void SetMutex (pthread_mutex_t Mutex)
  {
    _Mutex = Mutex;
  }

  void ConditionalWait ()
  {
    pthread_cond_wait (&_ConditionVar, &_Mutex);
  }

  int32_t ConditionalTimedWait (uint16_t TimeOutInSeconds = 5)
  {
    struct timespec ts;
    if (clock_gettime (CLOCK_REALTIME, &ts) == ERROR)
      {
	return ERROR;
      }
    ts.tv_sec += TimeOutInSeconds;

    return (pthread_cond_timedwait (&_ConditionVar, &_Mutex, &ts));
  }

  void ConditionalSignal ()
  {
    pthread_cond_signal (&_ConditionVar);
  }

private:
  pthread_cond_t _ConditionVar;
  pthread_mutex_t _Mutex;
};

#endif //_CONDITIONS_H

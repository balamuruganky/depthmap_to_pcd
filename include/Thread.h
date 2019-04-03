
#ifndef THREAD_H
#define THREAD_H

#include <pthread.h>

class Thread {
   public:
      virtual ~Thread() {}

      bool StartThread() {
         _ThreadStart = true;
         return (pthread_create(&_ThreadHandle, NULL, ThreadEntry, this) == 0);
      }

      void WaitForThreadToExit() {
         (void) pthread_join(_ThreadHandle, NULL);
         StopThread();
      }

      bool IsThreadStart()    { return _ThreadStart;    }
      void StopThread()       { _ThreadStart = false;   }

   protected:
      virtual void Run() = 0;

   private:
      static void*   ThreadEntry(void * This) { ((Thread *)This)->Run(); return NULL; }
      pthread_t      _ThreadHandle;
      volatile bool  _ThreadStart;
};

#endif // THREAD_H
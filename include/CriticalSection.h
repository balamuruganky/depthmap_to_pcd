#ifndef CRITICAL_SECTION_H
#define CRITICAL_SECTION_H

#include <pthread.h>

class CriticalSection {
	public:
		CriticalSection() {
			pthread_mutex_init( &_Mutex, NULL );
		}
		void Lock() {
			pthread_mutex_lock( &_Mutex );
		}
		void Unlock() {
			pthread_mutex_unlock( &_Mutex );
		}
		pthread_mutex_t GetObject() {
			return _Mutex;
		}
	private:
		pthread_mutex_t _Mutex;
};

#endif //CRITICAL_SECTION_H
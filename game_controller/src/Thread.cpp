using namespace std;

#include <pthread.h> // Should be the first include
#include <iostream>
#include "Thread.h"

/* Linux with glibc:
 *   _REENTRANT to grab thread-safe libraries
 */
#ifndef WIN32
#  ifndef _REENTRANT
#    define _REENTRANT
#  endif
#endif

Thread::Thread() : mArg(NULL) {
  running = false;
  mThreadId = pthread_self(); // Initialize mThreadId to a meaningful value
  if(pthread_mutex_init(&mRunningMutex, NULL)) {
    cerr << "[Thread] Could not initialize mutex!" << endl;
  }
}

Thread::~Thread() {
  stop();
  mArg = NULL;
  pthread_mutex_destroy(&mRunningMutex);
}

bool Thread::start(void *arg) {
  Arg(arg); // store user data
  int code = pthread_create(&mThreadId, NULL, &Thread::entryPoint, this);
  if (code > 0) {
    cerr << "[Thread] Start: pthread create error: " << code << endl;
    return false;
  }
  return true;
}

void Thread::stop() {
  void* status;
  pthread_mutex_lock(&mRunningMutex);
  if (running) {
    running = false;
    pthread_mutex_unlock(&mRunningMutex);
    terminate();
    // wait for thread to terminate
    pthread_join(mThreadId, (void **)&status);
  } else {
	pthread_mutex_unlock(&mRunningMutex);
  }
}

void Thread::Run(void *arg) {
  pthread_mutex_lock(&mRunningMutex);
  if (!running) {
    if (setup()) {
    	running = true;
    	pthread_mutex_unlock(&mRunningMutex);
    	execute(arg);
    } else {
    	pthread_mutex_unlock(&mRunningMutex);
    }
  } else {
	  pthread_mutex_unlock(&mRunningMutex);
  }
}

bool Thread::IsRunning() {
	pthread_mutex_lock(&mRunningMutex);
	bool ret = running;
	pthread_mutex_unlock(&mRunningMutex);
	return ret;
}

/*static */
void* Thread::entryPoint(void *pthis) {
  Thread *pt = static_cast<Thread*>(pthis);

  pt->Run(pt->Arg());
  return NULL;
}

bool Thread::setup() {
	// Do any setup here
	return true;
}

void Thread::terminate() {
  // Do any termination here
}

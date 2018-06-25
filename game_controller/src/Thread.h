#ifndef THREAD_H_
#define THREAD_H_

#include <pthread.h>

extern "C" {

/**
 * A simple thread interface for use of multi-threading.
 * Inherited classes only have to implement the execute method.
 * Then you can start and stop the thread when needed.
 */
class Thread {
  public:
    Thread();
    virtual ~Thread();

    /**
     * start the thread, so initialize and then call execute in a new thread
     * @param arg	user arguments
     * @return true if thread was started successfully
     */
    bool start(void *arg);

    /**
     * stop the thread and uninitialize everything
     */
    void stop();

  protected:
    /**
     * run
     * @param arg		arguments
     */
    void Run(void *arg);

    /**
     * entry point
     * @param pthis		point to this
     */
    static void* entryPoint(void *pthis);

    /**
     * arg
     */
    void *Arg() {return mArg;}

    /**
     * arg with arguments
     * @param a		arguments
     */
    void Arg(void *a) {mArg = a;}

    /**
     * is currently running
     */
    bool IsRunning();

  private:
    virtual bool setup();
    virtual void terminate();
    virtual void execute(void* arg) = 0;

    pthread_t mThreadId;
    pthread_mutex_t mRunningMutex;
    void *mArg;
    bool running;
};

}
#endif /*THREAD_H_*/

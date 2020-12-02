/************************************************
 * @file ThreadObject.cpp
 * @short creates/destroys a thread
 ***********************************************/

#include "ThreadObject.h"
#include "sls/container_utils.h"
#include <iostream>
#include <sys/syscall.h>
#include <unistd.h>

ThreadObject::ThreadObject(int threadIndex, std::string threadType,
                           bool do_start)
    : index(threadIndex), type(threadType) {
    sem_init(&semaphore, 1, 0);
    if (!do_start) {
        LOG(logINFO) << type << " thread skipped: " << index;
        return;
    }
    try {
        threadObject = std::thread(&ThreadObject::RunningThread, this);
    } catch (...) {
        throw sls::RuntimeError("Could not create " + type +
                                " thread with index " + std::to_string(index));
    }
    LOG(logDEBUG) << type << " thread created: " << index;
}

ThreadObject::~ThreadObject() {
    killThread = true;
    sem_post(&semaphore);
    if (threadObject.joinable())
        threadObject.join();
    sem_destroy(&semaphore);
}

pid_t ThreadObject::GetThreadId() const { return threadId; }

bool ThreadObject::IsRunning() const { return runningFlag; }

void ThreadObject::StartRunning() { runningFlag = true; }

void ThreadObject::StopRunning() { runningFlag = false; }

void ThreadObject::RunningThread() {
    threadId = syscall(SYS_gettid);
    LOG(logINFOBLUE) << "Created [ " << type << "Thread " << index
                     << ", Tid: " << threadId << "]";
    while (!killThread) {
        while (IsRunning()) {
            ThreadExecution();
        }
        // wait till the next acquisition
        sem_wait(&semaphore);
    }
    LOG(logINFOBLUE) << "Exiting [ " << type << " Thread " << index
                     << ", Tid: " << threadId << "]";
    threadId = 0;
}

void ThreadObject::Continue() { sem_post(&semaphore); }

void ThreadObject::SetThreadPriority(int priority) {
    if (!threadObject.joinable())
        return;
    struct sched_param param;
    param.sched_priority = priority;
    if (pthread_setschedparam(threadObject.native_handle(), SCHED_FIFO,
                              &param) == EPERM) {
        if (index == 0) {
            LOG(logWARNING) << "Could not prioritize " << type
                            << " thread. "
                               "(No Root Privileges?)";
        }
    } else {
        LOG(logINFO) << "Priorities set - " << type << ": " << priority;
    }
}

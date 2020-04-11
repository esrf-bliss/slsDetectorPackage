#pragma once
/************************************************
 * @file ThreadUtils.h
 * @short helper classes for thread synchronization
 ***********************************************/

#include <semaphore.h>
#include <pthread.h>

namespace FrameAssembler
{

class Semaphore
{
public:
	Semaphore(int n)
	{
		if (sem_init(&sem, 0, n) != 0)
			throw std::runtime_error("Could not init sem");
	}

	~Semaphore()
	{
		sem_destroy(&sem);
	}

	void post()
	{
		sem_post(&sem);
	}

	void wait()
	{
		sem_wait(&sem);
	}

private:
	sem_t sem;
};


class Cond;

/**
 * Mutex
 */

class Mutex
{
 public:
	Mutex()
	{
		if (pthread_mutex_init(&mutex, NULL) != 0)
			throw std::runtime_error("Could not init mutex");
	}

	~Mutex()
	{
		pthread_mutex_destroy(&mutex);
	}

	void lock()
	{
		if (pthread_mutex_lock(&mutex) != 0)
			throw std::runtime_error("Error locking mutex");
	}

	void unlock()
	{
		if (pthread_mutex_unlock(&mutex) != 0)
			throw std::runtime_error("Error unlocking mutex");
	}

 private:
	friend class Cond;
	pthread_mutex_t mutex;
};


/**
 * MutexLock
 */

class MutexLock
{
 public:
	MutexLock(Mutex& m)
		: mutex(m), locked(false)
	{ lock(); }

	~MutexLock()
	{ unlock(); }

	void lock()
	{
		if (locked)
			return;
		mutex.lock();
		locked = true;
	}

	void unlock()
	{
		if (!locked)
			return;
		mutex.unlock();
		locked = false;
	}

private:
	Mutex& mutex;
	bool locked;
};


class MutexUnlock
{
 public:
	MutexUnlock(MutexLock& l)
		: lock(l)
	{ lock.unlock(); }

	~MutexUnlock()
	{ lock.lock(); }

 private:
	MutexLock& lock;
};


/**
 * Cond
 */

class Cond
{
 public:
	Cond()
	{
		if (pthread_cond_init(&cond, NULL) != 0)
			throw std::runtime_error("Could not init cond");
	}

	~Cond()
	{
		pthread_cond_destroy(&cond);
	}

	Mutex& getMutex()
	{
		return mutex;
	}

	void wait()
	{
		if (pthread_cond_wait(&cond, &mutex.mutex) != 0)
			throw std::runtime_error("Could not wait on cond");
	}

	void signal()
	{
		if (pthread_cond_signal(&cond) != 0)
			throw std::runtime_error("Could not signal cond");
	}

	void broadcast()
	{
		if (pthread_cond_broadcast(&cond) != 0)
			throw std::runtime_error("Could not broadcast cond");
	}

 private:
	Mutex mutex;
	pthread_cond_t cond;
};


} // namespace FrameAssembler

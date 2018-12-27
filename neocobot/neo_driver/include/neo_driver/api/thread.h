#ifndef THREAD_H
#define THREAD_H

#ifdef _WIN32

#else
	#include <iostream>
	#include <pthread.h>
	#include <unistd.h>

	using namespace std;

	class Thread
	{
	private:
		pthread_t tid;
		int threadStatus;
		static void * thread_proxy_func(void * args);
		void* run1();
	public:
		static const int THREAD_STATUS_NEW = 0;
		static const int THREAD_STATUS_RUNNING = 1;
		static const int THREAD_STATUS_EXIT = -1;
		Thread();
		virtual void run() = 0;
		bool start();
		pthread_t getThreadID();
		int getState();
		void join();
		void join(unsigned long millisTime);
	};

#endif

#endif

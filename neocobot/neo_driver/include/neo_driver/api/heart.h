#ifndef HEART_H
#define HEART_H

#ifdef _WIN32
	#include "stdafx.h"
	#include "sendthread.h"
	#include "recvthread.h"

	class HeartThread
	{
	public:
		HeartThread();
		~HeartThread();
		void start();
		void stop();
		RobotEvent Load(SendThread *sendthread, RecvThread *recvthread);
		RobotEvent Unload();

	private:
		MyLogger *myLoger = NULL;

		SendThread *send = NULL;
		RecvThread *recv = NULL;

		HANDLE m_hThread;

		bool isRunning = false;
		bool isLogin = false;
		double startTime = 0.0;
		double stopTime = 0.0;
		double durationTime = 0.0;

		void Processing();
		static DWORD CALLBACK Sending(LPVOID);
		DWORD Proc();

	};

#else
	#include "stdafx.h"
	#include "sendthread.h"
	#include "recvthread.h"
	#include "thread.h"

	class HeartThread : public Thread
	{
	public:
		HeartThread();
		~HeartThread();
		void run();
		void stop();
		RobotEvent Load(SendThread *sendthread, RecvThread *recvthread);
		RobotEvent Unload();

	private:
		MyLogger *myLoger = NULL;

		SendThread *send = NULL;
		RecvThread *recv = NULL;

		bool isRunning = false;
		bool isLogin = false;
		bool isStop = false;
		double startTime = 0.0;
		double stopTime = 0.0;
		double durationTime = 0.0;

		void Processing();
	};

#endif

#endif


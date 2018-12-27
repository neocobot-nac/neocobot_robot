#ifndef SENDTHREAD_H
#define SENDTHREAD_H

#ifdef _WIN32
	#include <Winsock2.h>
	#pragma comment(lib, "ws2_32.lib")

	#include "stdafx.h"
	#include "safequeue.h"
	#include "tasks.h"
	#include "parser.h"

	using namespace std;

	class SendThread
	{
	public:
		SendThread();
		~SendThread();
		void start();
		void stop();
		RobotEvent Load(SOCKET &s);
		RobotEvent Unload();
		RobotEvent AddTask(Tasks task);

	private:
		MyLogger *myLoger = NULL;

		Parser *parser = new Parser;
		threadsafe_queue<Tasks> sendqueue;

		HANDLE m_hThread;

		SOCKET socket;

		bool isRunning = false;
		bool isLogin = false;

		void Processing();
		static DWORD CALLBACK Sending(LPVOID);
		DWORD Proc();
	};

#else
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <unistd.h>

	#include "stdafx.h"
	#include "safequeue.h"
	#include "tasks.h"
	#include "parser.h"
	#include "thread.h"

	using namespace std;

	class SendThread : public Thread
	{
	public:
		SendThread();
		~SendThread();
		void run();
		void stop();
		RobotEvent Load(int &s);
		RobotEvent Unload();
		RobotEvent AddTask(Tasks task);

	private:
		MyLogger *myLoger = NULL;

		Parser *parser = new Parser;
		threadsafe_queue<Tasks> sendqueue;

		int socket;

		bool isRunning = false;
		bool isLogin = false;

		void Processing();
	};

#endif

#endif


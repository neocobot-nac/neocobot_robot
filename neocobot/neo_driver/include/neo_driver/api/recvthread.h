#ifndef RECVTHREAD_H
#define RECVTHREAD_H

#define BUFLEN 1024

#ifdef _WIN32
	#include <atlbase.h>
	#include <atlsync.h>  
	#include <vector>
	#include <string.h>
	#include <Winsock2.h>
	#pragma comment(lib, "ws2_32.lib")

	#include "stdafx.h"
	#include "safemap.h"
	#include "parser.h"

	class RecvThread
	{
	public:
		RecvThread();
		~RecvThread();
		void start();
		void stop();
		RobotEvent Load(SOCKET &s);
		RobotEvent Unload();
		RobotEvent FastGetRecv(string index, map<string, string> &data);
		RobotEvent SlowGetRecv(string index, map<string, string> &data);

	private:
		MyLogger * myLoger = NULL;

		Parser *parser = new Parser;

		threadsafe_map<string, map<string, string>> fast_recvmap;
		threadsafe_map<string, map<string, string>> slow_recvmap;

		HANDLE m_hThread;

		char buf[BUFLEN];
		SOCKET socket;

		string buffer;

		bool isRunning = false;
		bool isLogin = false;

		void Processing();
		static DWORD CALLBACK Recving(LPVOID);
		DWORD Proc();
		RobotEvent generate_msg(string &msg);
	};

#else
	#include <vector>  
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>

	#include "stdafx.h"
	#include "safemap.h"
	#include "parser.h"
	#include "thread.h"

	class RecvThread : public Thread
	{
	public:
		RecvThread();
		~RecvThread();
		void run();
		void stop();
		RobotEvent Load(int &s_s);
		RobotEvent Unload();
		RobotEvent FastGetRecv(string index, map<string, string> &data);
		RobotEvent SlowGetRecv(string index, map<string, string> &data);

		threadsafe_map<string, map<string, string>> fast_recvmap;
		threadsafe_map<string, map<string, string>> slow_recvmap;
	private:
		MyLogger * myLoger = NULL;

		Parser *parser = new Parser;

		char buf[BUFLEN];
		int socket;

		string buffer;

		bool isRunning = false;
		bool isLogin = false;

		void Processing();
		RobotEvent generate_msg(string &msg);
	};

#endif

#endif

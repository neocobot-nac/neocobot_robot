/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     sender.h
*  @brief    消息发送线程模块
*
*  @detail   定义了消息发送模块的功能
*			 1.将任务转化为消息发送给服务器
*			 2.提供了事件回调的注册
*
*  @author   Huang Shaojie
*  @email    shaojie@neocobot.com
*  @version  0.2.0
*  @date     2019/01/17
*
*----------------------------------------------------------------------------
*  Change History :
*  <Date>     | <Version> | <Author>       | <Description>
*----------------------------------------------------------------------------
*  2019/01/17 | 0.2.0     | Huang Shaojie  | 重组并定义功能
*----------------------------------------------------------------------------
*****************************************************************************/
#ifndef _SENDER_H
#define _SENDER_H

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS 0
#include <Ws2tcpip.h>
#include <Winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <algorithm>
#include <sstream>
#endif

#include "argdefine.h"
#include "thread.h"
#include "tasks.h"
#include "safequeue.h"
#include "parser.h"
#include "logger.h"

/*
@brief 消息发送线程模块
*/
class Sender : public Neothread::NeoThread
{
private:
	/* 接收事件 */
	RobotEvent					_senderEvent;

	/* 发送模块存活标志与启动标志 */
	bool						_isRunning;
	bool						_isLoad;

	/* socket相关定义 */
	SOCKET						_socket;

	/* 任务打包模块 */
	Parser						_parser;

	/* 任务队列 */
	threadsafe_queue<Tasks>		_tasksQueue;

	/* 回调函数实例 */
	RobotEventCallback			_eventCallback;

private:
	/*
	@brief 事件回调函数
	*/
	void _eventCall();

public:
	Sender();
	~Sender();

	/*
	@brief 回调函数注册
	@param RobotEventCallback eventCallback 回调函数
	*/
	void RobotEventRegister(RobotEventCallback eventCallback);

	/*
	@brief 将已经连接的socket登陆到消息发送模块,开始发送
	*/
	void Load(SOCKET &socket);

	/*
	@brief socket登出,停止发送
	*/
	void Unload();

	/*
	@brief 将任务添加到任务队列中
	@param Tasks task 任务
	*/
	void AddTask(Tasks task);

	/*
	@brief 停止线程
	*/
	void Run();

	/*
	@brief 发送模块线程函数
	*/
	void Stop();
};


#endif

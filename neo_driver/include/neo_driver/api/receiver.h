/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     receiver.h
*  @brief    消息接收线程模块
*
*  @detail   定义了消息接收模块的功能
*			 1.接收服务器消息并转化为可读数据
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
#ifndef _RECEIVER_H
#define _RECEIVER_H

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
#include "safemap.h"
#include "parser.h"
#include "logger.h"

#define BUFLEN 1024

/*
@brief 消息接收线程模块
*/
class Receiver : public Neothread::NeoThread
{
private:
	/* 接收事件 */
	RobotEvent										_receiverEvent;

	/* 接收模块存活标志与启动标志 */
	bool											_isRunning;
	bool											_isLoad;

	/* socket相关定义 */
	SOCKET											_socket;
	char											_buf[BUFLEN];
	string											_buffer;

	/* 消息解析模块 */
	Parser											_parser;

	/* 返回消息的map容器 */
	threadsafe_map<string, map<string, string> >	_recvMap;

	/* 回调函数实例 */
	RobotEventCallback								_eventCallback;

private:
	/*
	@brief 消息生成函数,用于整理buffer中接收到的字符,并生成可读消息
	@return RobotEvent 事件
	*/
	RobotEvent	_GenerateMsg(string &msg);

	/*
	@brief 事件回调函数
	*/
	void		_eventCall();

public:
	Receiver();
	~Receiver();

	/*
	@brief 回调函数注册
	@param RobotEventCallback eventCallback 回调函数
	*/
	void		RobotEventRegister(RobotEventCallback eventCallback);

	/*
	@brief 通过索引查找消息容器中对应的消息
	@param string				index	消息索引
	@param map<string, string>	&data	获取到的消息
	@return RobotEvent 事件
	*/
	RobotEvent	GetRecv(string index, map<string, string> &data);

	/*
	@brief 将已经连接的socket登陆到消息接收模块,开始接收
	*/
	void		Load(SOCKET &socket);

	/*
	@brief socket登出,停止接收
	*/
	void		Unload();

	/*
	@brief 接收模块线程函数
	*/
	void		Stop();

	/*
	@brief 停止线程
	*/
	void		Run();

};


#endif


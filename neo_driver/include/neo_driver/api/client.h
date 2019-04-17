/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     client.h
*  @brief    socket管理模块头文件
*
*  @detail   目前版本的头文件定义了socket管理模块
*			 1.管理socket的开启和关闭
*            2.维护接受和发送线程
*            3.维护心跳包
*            4.管理底层事件
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
#ifndef _CLIENT_H
#define _CLIENT_H

#ifdef _WIN32
#define _WINSOCK_DEPRECATED_NO_WARNINGS 0
#include <Ws2tcpip.h>
#include <Winsock2.h>
#pragma comment(lib, "ws2_32.lib")
#include <string>
#else
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <algorithm>
#include <sstream>
#endif

#include "argdefine.h"
#include "sender.h"
#include "receiver.h"
#include "heart.h"
#include "logger.h"

/*
@brief socket管理模块
*/
class Client
{
private:
	/* socket参数定义 */
#ifdef _WIN32
	WSADATA					_wsa;
#else
	;
#endif

	struct sockaddr_in		_socketAddr;
	SOCKET					_socket;

	/* 事件记录 */
	RobotEvent				_clientEvent;

	/* 本地系统信息记录 */
	LocalSystemInfo			_localSystemInfo;

	/* 是否登陆标志 */
	bool					_isLogin;

	/* 发送,接收,心跳包线程模块实例化 */
	Receiver	receiver;
	Sender		sender;
	Heart		heart;

private:
	/* 事件回调函数与心跳包回调函数 */
	static void _eventCallback(RobotEvent event);
	static void _heartCallback();

	/*
	@brief 加载本地系统信息
	@return RobotEvent 返回事件
	*/
	RobotEvent _loadLocalSystemInfo();

public:
	/* 类指针,使回调函数可以使用类成员函数及参数 */
	static Client* pOdinClientCallback;

public:
	Client();
	~Client();

	/*
	@brief 添加任务
	@param Tasks task 任务
	@return RobotEvent 返回事件
	*/
	RobotEvent		AddTasks(Tasks task);

	/*
	@brief 加载本地系统信息
	@param string				name	需要获取的信息名字
	@param map<string, string>	&data	需要到的对应参数
	@return RobotEvent 返回事件
	*/
	RobotEvent		GetResult(string name, map<string, string> &data);

	/*
	@brief 初始化socket连接
	@param const char*	ServerIp	连接的服务器ip
	@param short int	ServerPort	连接的服务器端口
	@return RobotEvent 返回事件
	*/
	RobotEvent		InitializeSocket(const char* ServerIp, short int ServerPort);

	/*
	@brief 断开socket连接
	@return RobotEvent 返回事件
	*/
	RobotEvent		FinalizeSocket();

	/*
	@brief 获取本地ip
	@param string &ip 获取到的ip
	*/
	void			GetClientIP(string &ip);

	/*
	@brief 获取本地系统信息
	@return LocalSystemInfo 返回本地系统消息
	*/
	LocalSystemInfo	GetLocalSystemInfo();
};

#endif

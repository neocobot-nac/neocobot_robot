/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     heart.h
*  @brief    心跳包头文件
*
*  @detail   定义了心跳包的计数类, 10s一次心跳
*			 包含：
*			 1.心跳包线程的启动与停止
*			 2.心跳包计数,通过回调函数发送心跳包

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
#ifndef _HEART_H
#define _HEART_H

#include <time.h>
#include "argdefine.h"
#include "thread.h"
#include "parser.h"
#include "logger.h"

/* 心跳包回调函数类型定义 */
typedef void(*HeartEventCallback) (void);

/*
@brief 心跳包线程类
*/
class Heart : public Neothread::NeoThread
{
private:
	/* 心跳包存活标志 */
	bool						_isRunning;
	bool						_isEnable;

	/* 计数参数 */
	time_t						_startTime;
	time_t						_stopTime;
	double						_durationTime;

	/* 回调函数实例 */
	HeartEventCallback			_heartCallback;

private:
	/*
	@brief 事件回调函数
	*/
	void _eventCall();

public:
	Heart();
	~Heart();

	/*
	@brief 回调函数注册
	@param HeartEventCallback eventCallback 回调函数
	*/
	void HeartEventRegister(HeartEventCallback eventCallback);

	/*
	@brief 心跳包使能
	*/
	void Enable();

	/*
	@brief 心跳包失能
	*/
	void Disable();

	/*
	@brief 心跳包线程函数
	*/
	void Run();

	/*
	@brief 停止线程
	*/
	void Stop();
};


#endif

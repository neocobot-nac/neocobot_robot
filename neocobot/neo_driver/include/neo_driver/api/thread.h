/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     thread.h
*  @brief    线程模块
*
*  @detail   定义了基础线程模块的实现
*			 1.线程的启动停止实现
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
#ifndef _THREAD_H_
#define _THREAD_H_

#ifdef _WIN32
#include <Windows.h>
#else
#include <pthread.h>
#include <semaphore.h>
#endif

/* 线程命名空间定义 */
#ifndef _NEOTHREAD_BEGIN
#define _NEOTHREAD_BEGIN namespace Neothread {
#endif

#ifndef NEOCOBOT_THREAD
#define NEOCOBOT_THREAD Neothread
#endif

#ifndef _NEOTHREAD_END
#define _NEOTHREAD_END }
#endif

_NEOTHREAD_BEGIN

/*
@brief 线程类封装
通过继承该类并覆写Run()函数进行使用
Start调用将创建一个线程去执行Run函数.
Wait调用将阻塞当前调用者线程直到CThread线程退出.
Terminate调用将强制终止CThread线程, 改掉用应慎用!
*/
class NeoThread
{
public:
	NeoThread();
	virtual ~NeoThread();

public:
	bool Start();
	virtual void Run() = 0;
	bool Join();
	bool Detach();
	bool Terminate();

	//可通过对返回值强制转换为windows下的handle
	inline unsigned long long GetThread() { return m_hThread; };
private:
	unsigned long long m_hThread;
};

_NEOTHREAD_END

#endif  // _THREAD_H_

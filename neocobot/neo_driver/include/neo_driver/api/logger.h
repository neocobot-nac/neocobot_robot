/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     logger.h
*  @brief    日志记录接口
*
*  @detail   头文件定义了日志等级以及日志消息内容
*            使用线程监控日志消息并进行输出

*  @author   Huang Shaojie
*  @email    shaojie@neocobot.com
*  @version  0.2.0
*  @date     2019/01/17
*
*----------------------------------------------------------------------------
*  Change History :
*  <Date>     | <Version> | <Author>       | <Description>
*----------------------------------------------------------------------------
*  2019/01/17 | 0.2.0     | Huang Shaojie  | 组建并定义功能
*----------------------------------------------------------------------------
*****************************************************************************/
#ifndef _LOGGER_H
#define _LOGGER_H

#include <string>
#include <fstream>
#include <iostream>
#include <stdarg.h>

#include "argdefine.h"
#include "thread.h"
#include "safequeue.h"

using std::ofstream;
using std::cout;
using std::endl;
using std::to_string;
using std::string;

#if defined(WINDOWS) || defined(_WIN32)
#ifndef _DEBUG
#define _DEBUG 0
#endif

#else
;
#endif

/*
@brief 日志类型定义
*/
enum class LogType {
	Debug,
	Info,
	Warning,
	Error,
	Fatal
};

/*
@brief 日志等级定义
*/
enum class LogLevel {
	NONE,
	LOW,
	MEDIUM,
	HIGH,
	CRITICAL
};

/*
@brief 日志消息定义
*/
typedef struct log_message
{
private:
	LogType				_type;		// 类型
	LogLevel			_level;		// 等级
	RobotEvent			_event;		// 错误事件
	string				_message;	// 日志消息

	/*
	@brief 将日志类型转化为对应的字符串
	@return string 日志类型字符串
	*/
	string LogTypeStr()
	{
		switch (_type)
		{
		case LogType::Debug:
			return "DEBUG";
		case LogType::Info:
			return "INFO";
		case LogType::Warning:
			return "WARNING";
		case LogType::Fatal:
			return "FATAL";
		case LogType::Error:
		default:
			return "ERROR";
		}
	}

public:
	log_message(LogType type, LogLevel level, RobotEvent event, const char* message) :
		_type(type), _level(level), _event(event), _message(message) {};
	log_message() {};
	~log_message() {};

	/*
	@brief 输出日志信息转化为可输出形式
	@return string 可输出字符串
	*/
	string format()
	{
		switch (_type)
		{
		case LogType::Debug:
		case LogType::Info:
			return LogTypeStr() + ": " + _message;
		case LogType::Warning:
		case LogType::Error:
		case LogType::Fatal:
		default:
			return LogTypeStr() + " " + +"(" + to_string(static_cast<unsigned int>(_event)) + ")" + ": " + _message;
		}
	}
}LogMessage, *pLogMessage;


/*
@brief 日志记录
*/
class NeoLogger : Neothread::NeoThread
{
private:
	/* 日志消息队列 */
	threadsafe_queue<LogMessage> _MsgQueue;

	bool isRunning;

private:
	NeoLogger() :NeoThread() {};
	~NeoLogger() {};

public:
	void operator=(NeoLogger const&) = delete;
	NeoLogger(NeoLogger const&) = delete;

	/*
	@brief 获得实例
	*/
	static NeoLogger& GetInstance()
	{
		static NeoLogger _instance;
		return _instance;
	}

	/*
	@brief 开启日志服务
	*/
	static RobotEvent StartLog()
	{
		NeoLogger& _logger = GetInstance();
		_logger.isRunning = true;
		_logger.Start();
		return RobotEvent_Succeed;
	}

	/*
	@brief 实现NeoThread线程类的Run方法
	*/
	virtual void Run()
	{
		time_t rawtime;
		struct tm* timeinfo;
		char logformat[32];

		// 仅在正常运行模式下才会以文件形式输出日志信息
#ifdef _DEBUG
#else
		// 获取日志路径
		char fileformat[32];
		string filepath;

		RobotEvent fret = NeoResource::Get(NeoResource::LOG_DIR, &filepath);
		if (RobotEvent_Succeed == fret)
		{
			time(&rawtime);
			timeinfo = localtime(&rawtime);
			strftime(fileformat, sizeof(fileformat), "%Y%m%d%H%M%S", timeinfo);

			filepath.append(PATH_OPERATOR);
			filepath.append(fileformat);
			filepath.append(".log");

			ofstream fout(filepath);
#endif
			while (isRunning)
			{
				LogMessage _msg;
				_MsgQueue.wait_and_pop(_msg);

				time(&rawtime);
				timeinfo = localtime(&rawtime);
				strftime(logformat, sizeof(logformat), "%Y-%m-%d %H:%M:%S", timeinfo);

				string err = _msg.format();
#ifdef _DEBUG
				printf("%s  %s\n", logformat, err.c_str());
#else
				fout << logformat << " " << err << endl;
#endif
			}

#ifdef _DEBUG
#else
			fout.close();
		}
#endif
	}

	/*
	@brief 调试信息，仅在Debug模式下才会输出
	@param string message Debug模式输出信息
	*/
	static void Debug(const char* message, ...)
	{
#ifdef _DEBUG
		va_list args;
		va_start(args, message);

		NeoLogger& _logger = GetInstance();
		string print_msg = _logger.LogPrint(message, args);
		LogMessage* _msg = new LogMessage(LogType::Debug, LogLevel::NONE, RobotEvent_Succeed, print_msg.c_str());
		_logger._MsgQueue.push(*_msg);

		va_end(args);
#endif
	}

	/*
	@brief 日志正常信息输出
	@param string message 输出信息
	*/
	static void Info(const char* message, ...)
	{
		va_list args;
		va_start(args, message);

		NeoLogger& _logger = GetInstance();
		string print_msg = _logger.LogPrint(message, args);
		LogMessage* _msg = new LogMessage(LogType::Info, LogLevel::NONE, RobotEvent_Succeed, print_msg.c_str());
		_logger._MsgQueue.push(*_msg);

		va_end(args);
	}

	/*
	@brief 日志警告信息输出，一般不影响正常运行可以自我修复的用警告报出
	@param RobotEvent	event  错误码
	@param string		message 错误(警告)详细信息
	*/
	static void Warning(RobotEvent event, const char* message, ...)
	{
		va_list args;
		va_start(args, message);

		NeoLogger& _logger = GetInstance();
		string print_msg = _logger.LogPrint(message, args);
		LogMessage* _msg = new LogMessage(LogType::Warning, LogLevel::NONE, event, print_msg.c_str());
		_logger._MsgQueue.push(*_msg);

		va_end(args);
	}

	/*
	@brief 日志错误信息输出，一般影响到部分功能运行但不至于让程序崩溃或退出的以错误形式报出
	@param LogLevel		level   报错级别，分为LOW/MEDIUM/HIGH/CRITICAL四种等级
	@param RobotEvent	event	错误码
	@param string		message 运行错误详细信息
	*/
	static void Error(LogLevel level, RobotEvent event, const char* message, ...)
	{
		va_list args;
		va_start(args, message);

		NeoLogger& _logger = GetInstance();
		string print_msg = _logger.LogPrint(message, args);
		LogMessage* _msg = new LogMessage(LogType::Error, level, event, print_msg.c_str());
		_logger._MsgQueue.push(*_msg);

		va_end(args);
	}

	/*
	@brief 日志失败信息输出，一般情况下最严重的错误导致程序无法进行或崩溃会以Fatal形式报出警报
	@param RobotEvent	event	错误码
	@param string		message 运行失败详细信息
	*/
	static void Fatal(RobotEvent event, const char* message, ...)
	{
		va_list args;
		va_start(args, message);

		NeoLogger& _logger = GetInstance();
		string print_msg = _logger.LogPrint(message, args);
		LogMessage* _msg = new LogMessage(LogType::Fatal, LogLevel::CRITICAL, event, print_msg.c_str());
		_logger._MsgQueue.push(*_msg);

		va_end(args);
	}

	/*
	@brief 日志消息转化,格式化输出
	@param const char*	message	日志消息
	@param va_list		args	可变参数列表
	@return string 格式化后的字符串
	*/
	string LogPrint(const char* message, va_list args)
	{
		string	s_log;
		string	arg_type;
		char*	str_tmp = NULL;
		bool	isfind = false;

		while (*message != '\0')
		{
			switch (*message)
			{
			case '%':
				isfind = true;
				arg_type.clear();
				break;

			case 'u':
			case 'd':
				if (isfind)
				{
					isfind = false;
					s_log.append(to_string(va_arg(args, int)));
					break;
				}

			case 'f':
				if (isfind)
				{
					isfind = false;
					s_log.append(to_string(va_arg(args, double)));
					break;
				}

			case 'c':
				if (isfind)
				{
					isfind = false;
					s_log.append(to_string(va_arg(args, int)));
					break;
				}

			case 's':
				if (isfind)
				{
					isfind = false;
					str_tmp = (char*)va_arg(args, int);
					while (*str_tmp != '\0')
					{
						s_log.append(1, *str_tmp);
						str_tmp++;
					}
					break;
				}

			default:
				s_log.append(1, *message);
				break;
			}
			if (isfind)
			{
				arg_type.append(1, *message);
			}
			message++;
		}

		return s_log;
	}
};

#endif

/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     tools.h
*  @brief    常用工具函数定义
*
*  @detail   定义了目前使用到的工具函数或工具类
*			 包含：
*			 1.消息的计数时间戳,可用于区分消息
*			 2.timesleep函数
*			 3.将字符串解析为机械臂事件
*			 4.文件夹的创建
*			 5.重定向文件的加载器
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
#ifndef _TOOLS_H
#define _TOOLS_H

#ifdef _WIN32
#include <windows.h>
#include <io.h>
#include <direct.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif

#include <algorithm>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>
#include <fstream>

#include "neoclient.h"

using std::string;
using std::vector;
using std::multimap;
using std::ifstream;
using std::ptr_fun;
using std::not1;

/* 跨平台文件获取和创建定义 */
#if defined(WINDOWS) || defined(_WIN32)
#define ACCESS(fileName,accessMode) _access(fileName,accessMode)
#define MKDIR(path) _mkdir(path)
#else
#define ACCESS(fileName,accessMode) access(fileName,accessMode)
#define MKDIR(path) mkdir(path,S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH)
#endif

/* 最大路径长度定义 */
#define MAX_PATH_LEN	256

/* 最大时间戳 */
#define MAX_STAMP		10240

/*
@brief 获取时间戳
@return int 返回时间戳
*/
int	GetTimeStamp();

/*
@brief 休眠函数
@param double sec 需要休眠的事件,单位为秒
*/
void TimeSleep(double sec);

/*
@brief 字符串分割
@param string result 输入的结果字符串
@return RobotEvent 对应的事件
*/
RobotEvent GetRobotEvent(string result);

/*
@brief 创建目录
@param const string &directoryPath 需要创建的路径
@return RobotEvent 事件
*/
RobotEvent createDirectory(const string &directoryPath);

/*
@brief 重定向文件加载
*/
class NeoProperties
{
private:
	string						path;
	vector<string>				vLine;
	multimap<string, string>	msKV;
	bool						mulremark = false;

private:
	/*
	@brief 去除多余空格
	@param string &s 需要去除空格的字符串
	*/
	void trim(string &str);

public:
	NeoProperties() {};
	virtual ~NeoProperties() {};

	/*
	@brief 打开重定向文件
	@param const char* path 文件目录
	@return RobotEvent 事件
	*/
	RobotEvent open(const char* path);

	/*
	@brief 加载内容
	@return RobotEvent 事件
	*/
	RobotEvent load();

	/*
	@brief 关闭文件
	*/
	void close();

	/*
	@brief 根据路径关键字加载路径
	@param const char	*k		关键字
	@param string*		path	路径
	@return RobotEvent 事件
	*/
	RobotEvent read(const char *k, string* path);
};

#endif


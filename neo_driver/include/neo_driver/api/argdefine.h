/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     argdefine.h
*  @brief    常用定义及类型
*
*  @detail   定义了目前使用到的数据类型以及文件路径源
*			 包含：
*			 1.运动模式的定义
*			 2.定向配置文件(独例模式）
*			 3.预定义了未来版本中可能出现的运动路径相关数据类型
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
#ifndef _ARGDEFINE_H
#define _ARGDEFINE_H

#include "tools.h"
#include "neoclient.h"

/* 路径分割符定义 */
#if defined(WINDOWS) || defined(_WIN32)
#define PATH_OPERATOR "\\"
#else
#define PATH_OPERATOR "/"
#endif

/* 路径名称定义 */
#define PROPERTIES_FILE		"NeoClientResource.properties"
#define NEOCLIENT_PATH		"NeoClient"
#define LOG_PATH			            "log"
#define REPOSITORY_PATH		"repository"

/* 路径类型定义 */
typedef unsigned int DirName;

/* 常用类型定义 */
#if defined(WINDOWS) || defined(_WIN32)
;
#else
#define SOCKET_ERROR -1
typedef int SOCKET;
#endif

/* 回调函数类型定义 */
typedef void(*RobotEventCallback) (RobotEvent);

/*
@brief 配置连接信息（暂时不提供更改）
*/
typedef struct Setup_Param
{
	const char* channel_name = "/dev/pcanusb32";
	const char* channel_type = "PEAK_SYS_PCAN_USB";
	const char* protocol = "TMLCAN";
	int host_id = 10;
	int baud_rate = 500000;

}SetupParam;

/*
@brief 机械臂配置模式
SetupMode_RealMode表示真实连接
SetupMode_VirtualMode表示虚拟连接
*/
typedef enum Setup_Mode
{
	SetupMode_RealMode		= 0,
	SetupMode_VirtualMode	= 1

}SetupMode;

/*
@brief 机械臂运动模式
RelativeMode_Absolute表示绝对位置运动
RelativeMode_Relative表示相对位置运动
*/
typedef enum Relative_Mode
{
	RelativeMode_Absolute = 0,
	RelativeMode_Relative = 1

}RelativeMode;

/*
@brief 机械臂路径闭环模式
TrajCloseMode_Closed表示路径闭环
TrajCloseMode_Open表示路径开环
*/
typedef enum Traj_Close_Mode
{
	TrajCloseMode_Closed	= 0,
	TrajCloseMode_Open		= 1

}TrajCloseMode;

/*
@brief 本地系统信息定义
*/
typedef struct Local_System_Info
{
	const char* version;
	const char* system;
	const char* language;
}LocalSystemInfo;

/*
@brief 机械臂系统信息定义
*/
typedef struct Robot_System_Info
{
	const char* version;
	const char* system;
	const char* language;
}RobotSystemInfo;

/*
@brief 机械臂末端位置定义
*/
typedef struct _Position
{
	double x;
	double y;
	double z;

}Position;

/*
@brief 机械臂末端姿态定义
*/
typedef struct _Orientation
{
	double Rx;
	double Ry;
	double Rz;

}Orientation;

/*
@brief 机械臂末端位姿定义
*/
typedef struct _Pose
{
	Position position;
	Orientation orientation;

}Pose;

/*
@brief 机械臂文件路径源配置
*/
class NeoResource
{
public:
	bool isSetup = false;

	string _userPath;
	string _neoclientPath;
	string _logPath;
	string _repositoryPath;

	const static DirName USER_DIR = 0x0001U;
	const static DirName NEOCLIENT_DIR = 0x0002U;
	const static DirName LOG_DIR = 0x0003U;
	const static DirName REPOSITORY_DIR = 0x0004U;

public:
	NeoResource() {};
	~NeoResource() {};

	void operator=(NeoResource const&) = delete;
	NeoResource(NeoResource const&) = delete;

	/*
	@brief 获得实例
	*/
	static NeoResource& GetInstance()
	{
		static NeoResource _instance;
		return _instance;
	}

	/*
	@brief 加载路径,无重定向文件则路径默认在用户路径下
	@return RobotEvent 加载事件
	*/
	static RobotEvent Load()
	{
		NeoResource& _resource = GetInstance();

#if defined(WINDOWS) || defined(_WIN32)
		_resource._userPath = std::getenv("USERPROFILE");
#else
        _resource._userPath = std::getenv("HOME");
#endif
		/* 加载重定向文件 */
		RobotEvent ret_properties;
		NeoProperties _properties;
		ret_properties = _properties.open(PROPERTIES_FILE);
		if (RobotEvent_Succeed == ret_properties)
			_properties.load();

		/* 获取路径 */
		RobotEvent ret_createDir;
		ret_createDir = RobotEvent_Succeed == ret_properties ? _properties.read("NEOCLIENT_PATH", &_resource._neoclientPath) : RobotEvent_ConfigError;
		if (RobotEvent_ConfigError == ret_createDir)
			_resource._neoclientPath = _resource._userPath + PATH_OPERATOR + NEOCLIENT_PATH;

		ret_createDir = createDirectory(_resource._neoclientPath);
		if (RobotEvent_Succeed != ret_createDir)
		{
			_properties.close();
			return ret_createDir;
		}

		ret_createDir = RobotEvent_Succeed == ret_properties ? _properties.read("LOG_PATH", &_resource._logPath) : RobotEvent_ConfigError;
		if (RobotEvent_ConfigError == ret_createDir)
			_resource._logPath = _resource._neoclientPath + PATH_OPERATOR + LOG_PATH;

		ret_createDir = createDirectory(_resource._logPath);
		if (RobotEvent_Succeed != ret_createDir)
		{
			_properties.close();
			return ret_createDir;
		}

		ret_createDir = RobotEvent_Succeed == ret_properties ? _properties.read("REPOSITORY_PATH", &_resource._repositoryPath) : RobotEvent_ConfigError;
		if (RobotEvent_ConfigError == ret_createDir)
			_resource._repositoryPath = _resource._neoclientPath + PATH_OPERATOR + REPOSITORY_PATH;

		ret_createDir = createDirectory(_resource._repositoryPath);
		if (RobotEvent_Succeed != ret_createDir)
		{
			_properties.close();
			return ret_createDir;
		}

		_resource.isSetup = true;
		return RobotEvent_Succeed;
	}

	/*
	@brief 获取路径
	@param DirName name 路径对应名称
	@param string* path	返回路径值
	@return RobotEvent 获取事件
	*/
	static RobotEvent Get(DirName name, string* path)
	{
		NeoResource& _resource = GetInstance();

		switch (name)
		{
		case NeoResource::USER_DIR:
			*path = _resource._userPath;
			break;
		case NeoResource::NEOCLIENT_DIR:
			*path = _resource._neoclientPath;
			break;
		case NeoResource::LOG_DIR:
			*path = _resource._logPath;
			break;
		case NeoResource::REPOSITORY_DIR:
			*path = _resource._repositoryPath;
			break;
		default:
			return RobotEvent_ConfigError;
		}

		return RobotEvent_Succeed;
	}
};

#endif

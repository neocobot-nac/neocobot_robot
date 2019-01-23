/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     neoclient.h
*  @brief    NeoClient全局定义及系统配置参数
*
*  @detail   目前版本的头文件定义了NeoClient基本声明
*            1.内部事件错误及对应错误代码
*            2.命名空间声明
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
#ifndef _NEOCLIENT_H
#define _NEOCLIENT_H

/* NeoClient命名空间定义 */
#ifndef _NEOCLIENT_BEGIN
#define _NEOCLIENT_BEGIN namespace NeoClient {
#endif

#ifndef NEOCLIENT
#define NEOCLIENT NeoClient
#endif

#ifndef _NEOCLIENT_END
#define _NEOCLIENT_END }
#endif

/*
@brief 机械臂事件定义,包含对应错误码
*/
typedef enum Robot_Event
{
	RobotEvent_Succeed						= 0,			// 无错误

	// Client
	RobotEvent_SendError					= 1001011,		// 发送错误

	RobotEvent_RecvError					= 1002011,		// 接收错误
	RobotEvent_GetRecvError					= 1002021,		// 获取接收错误

	RobotEvent_GenerateFailed				= 1003011,		// 生成消息错误
	RobotEvent_UnwrapFailed					= 1003021,		// 消息解包错误
	RobotEvent_RecvTimeout					= 1003031,		// 接收超时错误

	RobotEvent_InitSocketError				= 1004011,		// socket初始化错误
	RobotEvent_SocketConnectError			= 1004021,		// socket连接错误

	RobotEvent_ConfigError					= 1005011,		// 加载配置错误

	// Server
	RobotEvent_FunctionNotFound				= 1001010,		// 无法找到对应函数
	RobotEvent_Version_Mismatch				= 1002010,		// 版本不匹配
	RobotEvent_UnexpectedArgs				= 1003010,		// 参数不匹配
	RobotEvent_RobotNotInit					= 1004010,		// 机械臂未初始化
	RobotEvent_RobotAlreadyInit				= 1004020,		// 机械臂已经初始化
	RobotEvent_RobotError					= 1004030,		// 机械臂功能执行错误

	RobotEvent_NonstandradRegisterParams	= 2001010,		// 注册参数不标准
	RobotEvent_RegisterError				= 2001020,		// 注册不成功
	RobotEvent_NotRegister					= 2002010,		// 未注册
	RobotEvent_NonstandradInfoParams		= 2002020,		// 连接参数不标准
	RobotEvent_NotConnected					= 2003010,		// 连接失败

	RobotEvent_ConnLackHead					= 3001010,		// 连接消息缺少消息头
	RobotEvent_HeadLackIdentifier			= 3001020,		// 连接消息缺少身份标签
	RobotEvent_IdentifierNotMatch			= 3001030,		// 身份标签不匹配
	RobotEvent_IdentifierNotFound			= 3001040,		// 身份标签缺失
	RobotEvent_ConnLackTail					= 3001050,		// 连接消息缺少消息尾
	RobotEvent_CtoSLackHead					= 3001060,		// 任务消息缺少消息头
	RobotEvent_HeadLackElement				= 3001070,		// 消息头缺少元素
	RobotEvent_CtoSLackFunction				= 3001080,		// 任务消息缺少任务标签
	RobotEvent_FunctionLackName				= 3001090,		// 任务标签缺少任务名称
	RobotEvent_FunctionLackParams			= 3001100,		// 任务标签缺少任务参数
	RobotEvent_CtoSLackTail					= 3001110,		// 任务消息缺少消息尾
	RobotEvent_ConnCheckSumFailed			= 3002010,		// 连接消息和校验失败
	RobotEvent_CtoSCheckSumFailed			= 3002020,		// 任务消息和校验失败
	RobotEvent_UncognizedTask				= 3002020,		// 无法识别的任务

	RobotEvent_SocketTimeout				= 4001010,		//通讯超时

}RobotEvent;


#endif


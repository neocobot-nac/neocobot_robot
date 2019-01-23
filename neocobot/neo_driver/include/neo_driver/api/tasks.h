/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     task.h
*  @brief    任务类别定义
*
*  @detail   定义了任务类别
*			 1.任务的种类
*			 2.任务包含的消息
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
#ifndef TASKS_H
#define TASKS_H

#include <map>
using std::map;

/*
@brief 定义任务类型
*/
typedef enum
{
	TASK_CONN = 100,
	TASK_C2S,
	TASK_HEART

}TaskType;

/*
@brief 任务定义
*/
class Tasks
{
public:
	TaskType type;
	map<string, string> data;

	Tasks() 
	{
		data.clear();
	}

	~Tasks() 
	{
		data.clear();
	}
};

#endif
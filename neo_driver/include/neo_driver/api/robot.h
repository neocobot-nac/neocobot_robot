/*****************************************************************************
*  Neocobot OdinClient library
*  Copyright (C) 2019 Huang Shaojie  shaojie@neocobot.com
*
*  @file     robot.h
*  @brief    OdinClient用户接口
*
*  @detail   目前版本的头文件定义了OdinClient用户接口
*            定义的接口包括如下内容:
*			 1. RobotLogin			登陆服务器
*			 2. RobotLogout			登出服务器
*			 3. Setup				机械臂初始化配置
*			 4. Shutdown			关闭机械臂
*			 5. move_to_angles		关节角度运动
*			 6. move_to_pose		末端位姿运动
*			 7. wait_for_motors		等待运动结束
*			 8. moveJ				关节轨迹运动
*			 9. moveL				多边形轨迹运动
*			 10. moveP				复杂轨迹运动
*			 11. release			松开机械臂关节
*			 12. hold				抱紧机械臂关节
*			 13. stop				停止当前运动并进入不可运动状态
*			 14. recover			恢复可运动状态
*			 15. calibrate			校准
*			 16. reset				复位
*			 17. get_motor_ids		获取关节ID
*			 18. get_angles			获取关节角度
*			 19. get_velocity		获取关节速度
*			 20. get_current		获取关节电流
*			 21. get_pose			获取末端位姿
*			 22. forward			前向运动学解算
*			 23. inverse			逆向运动学解算
*			 24. get_input			获取IO信息
*			 25. set_output			设置IO信息
*
*  @author   Huang Shaojie
*  @email    shaojie@neocobot.com
*  @version  0.2.0
*  @date     2019/01/14
*
*----------------------------------------------------------------------------
*  Change History :
*  <Date>     | <Version> | <Author>       | <Description>
*----------------------------------------------------------------------------
*  2019/01/14 | 0.2.0     | Huang Shaojie  | 重构v0.1.0,添加注释
*----------------------------------------------------------------------------
*****************************************************************************/

#ifndef _ROBOT_H
#define _ROBOT_H

#include "client.h"

/* 字符串转化中的符号说明 */
#define LEFT_BRACES			"{"
#define RIGHT_BRACES		"}"
#define LEFT_BRACKET		"["
#define RIGHT_BRACKET		"]"
#define DOT					","
#define QUOTATION			"\""
#define COLON				":"
#define SPACE				" "

/* 声明当前可支持的最大电机数 */
#define MAX_MOTORS		10

/*
@brief NeoClient接口类
对外负责提供用户接口,对内生成接口对应的任务并处理任务返回值
*/
class Robot
{
private:
	/* 消息管理器实例 */
	Client client;

	/* 当前连接机械臂的电机编号的实例化数组与电机数量 */
	vector<int>			_motorIds;
	size_t				_idSize;

	/* 用户IP */
	string				ip;

	/* 机械臂系统信息 */
	RobotSystemInfo		_robotSystemInfo;

	/* 机械臂的初始化配置状态,true表示已经配置,false反之 */
	bool				_isSetup;

private:
	/*
	@brief 获取电机编号与数量并载入_motorIds与_idSize实例化参数中
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 载入_motorIds与_idSize实例化参数中
	*/
	RobotEvent _getMotorIds();

	/*
	@brief 将SetupParan参数转化为string
	@param SetupParam	parameters	机械臂通讯配置参数
	@param string&		_setupParam	转化后的字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_SetupParamString(SetupParam parameters, string& _setupParam);

	/*
	@brief 将电机编号参数转化为string
	@param vector<int>	id_params		机械臂电机编号容器
	@param string&		string_params	转化后的字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_IdString(vector<int> id_params, string& string_params);

	/*
	@brief 将电机编号参数转化为string
	@param const int	id_params		机械臂电机编号
	@param string&		string_params	转化后的字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_IdString(const int id_params, string& string_params);

	/*
	@brief 将电机编号参数转化为string
	@param size_t		size			电机编号数量
	@param const int*	id_params		一组机械臂电机编号
	@param string&		string_params	转化后的字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_IdString(size_t size, const int* id_params, string& string_params);

	/*
	@brief 将int参数转化为string
	@param const int	int_params		int参数
	@param string&		string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_IntString(const int int_params, string& string_params);

	/*
	@brief 将int参数转化为string
	@param size_t		size			参数数量
	@param const int*	int_params		int参数
	@param string&		string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_IntString(size_t size, const int* int_params, string& string_params);

	/*
	@brief 将double参数转化为string
	@param const double	double_params	double参数
	@param string&		string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_DoubleString(const double double_params, string& string_params);

	/*
	@brief 将double参数转化为string
	@param size_t			size			参数数量
	@param const double*	double_params	double参数
	@param string&			string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_DoubleString(size_t size, const double* double_params, string& string_params);

	/*
	@brief 将io信号参数转化为string
	@param const int		id_params		电机编号数量
	@param const int		signal_params	io信号参数
	@param string&			string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_SignalString(const int id_params, const int signal_params, string& string_params);

	/*
	@brief 将io信号参数转化为string
	@param size_t			size			电机编号数量
	@param const int*		id_params		一组电机编号
	@param const int*		signal_params	一组io信号参数
	@param string&			string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_SignalString(size_t size, const int* id_params, const int* signal_params, string& string_params);

	/*
	@brief 将动作参数转化为string
	@param Pose				pose_param		动作参数
	@param string&			string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_PoseString(Pose pose_param, string& string_params);

	/*
	@brief 将动作参数转化为string
	@param size_t			size			动作参数数量
	@param Pose				pose_param		一组动作参数
	@param string&			string_params	转化为字符串
	@step 取出参数内容
	@step 字符串拼接
	*/
	void to_PointString(size_t size, Pose* point_param, string& string_params);

	/*
	@brief 字符串分割
	@param const string&	origin_params	源字符
	@param vector<string>&	split_params	分割后的字符
	@param const string&	key_params		分割关键字符
	@step 查找字符
	@step 前向分割后存入分割后的字符
	*/
	void SplitString(const string& origin_params, vector<string>& split_params, const string& key_params);

public:
	/*
	@brief Robot构造函数
	@step 初始化数据
	*/
	Robot();

	/*
	@brief Robot析构函数
	@step 清空数据
	*/
	~Robot();

	/*
	@brief 登陆服务器建立连接
	@param const char*	ServerIp	服务器ip地址,格式为xxx.xxx.xxx
	@param short int	ServerPort	服务器端口,数值范围为0-65535
	@return RobotEvent 机械臂事件
	@step 初始化socket配置
	@step 生成注册任务
	@step 获取任务返回值
	@step 判断注册是否成功
	@step 生成获取信息任务
	@step 获取任务返回值
	@step 记录获取的服务器系统信息
	@step 使用_getMotorIds方法初始化配置
	*/
	RobotEvent RobotLogin(const char* ServerIp, short int ServerPort);

	/*
	@brief 登出服务器
	@return RobotEvent 机械臂事件
	@step 关闭socket
	*/
	RobotEvent RobotLogout();

	/*
	@brief 配置机械臂并初始化
	@param const char*	name		机械臂名称
	@param SetupParam	parameters	连接参数
	@param SetupMode	mode		连接模式,SetupMode_RealMode为连接实际机械臂,SetupMode_VirtualMode为连接虚拟机械臂
	@return RobotEvent 机械臂事件
	@step 生成机械臂初始化任务
	@step 获取任务返回值
	@step 查验是否配置成功
	*/
	RobotEvent Setup(const char* name, SetupParam parameters, SetupMode mode);

	/*
	@brief 配置机械臂并初始化
	@return RobotEvent 机械臂事件
	@step 生成结束连接任务
	@step 获取任务返回值
	@step 查验是否结束成功
	*/
	RobotEvent Shutdown();

	/*
	@brief 关节角度运动
	@param const int		motor_ids		电机编号
	@param const double		angles			编号对应的角度值
	@param float			velocity		运动最大速度值(默认值为30.0)
	@param float			acceleration	运动最大加速度值(默认值为150.0)
	@param RelativeMode		relative		关联模式,RelativeMode_Absolute表示当前角度值为绝对角度,RelativeMode_Relative表示当前角度值为角度增量(默认值为RelativeMode_Absolute)
	@return RobotEvent 机械臂事件
	@step 生成关节角度运动任务
	@step 获取任务返回值
	@step 查验是否接收成功
	*/
	RobotEvent move_to_angles(const int motor_ids, const double angles, float velocity = 30.0, float acceleration = 150.0, RelativeMode relative = RelativeMode_Absolute);
		
	/*
	@brief 关节角度运动
	@param size_t			size			运动的电机数量
	@param const int*		motor_ids		一组电机编号
	@param const double*	angles			一组与编号对应的角度值
	@param float			velocity		运动最大速度值(默认值为30.0)
	@param float			acceleration	运动最大加速度值(默认值为150.0)
	@param RelativeMode		relative		关联模式,RelativeMode_Absolute表示当前角度值为绝对角度,RelativeMode_Relative表示当前角度值为角度增量(默认值为RelativeMode_Absolute)
	@return RobotEvent 机械臂事件
	@step 生成关节角度运动任务
	@step 获取任务返回值
	@step 查验是否接收成功任务
	*/
	RobotEvent move_to_angles(size_t size, const int* motor_ids, const double* angles, float velocity = 30.0, float acceleration = 150.0, RelativeMode relative = RelativeMode_Absolute);

	/*
	@brief 末端位姿运动
	@param Pose				pose			末端位姿，包括位置值(x,y,z),姿态值(Rx,Ry,Rz)
	@param float			velocity		运动最大速度值(默认值为30.0)
	@param float			acceleration	运动最大加速度值(默认值为150.0)
	@param RelativeMode		relative		关联模式,RelativeMode_Absolute表示当前角度值为绝对角度,RelativeMode_Relative表示当前角度值为角度增量(默认值为RelativeMode_Absolute)
	@return RobotEvent 机械臂事件
	@step 生成末端位姿运动任务
	@step 获取任务返回值
	@step 查验是否接收成功任务
	*/
	RobotEvent move_to_pose(Pose pose, float velocity = 30.0, float acceleration = 150.0, RelativeMode relative = RelativeMode_Absolute);

	/*
	@brief 等待电机运动结束
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否运动结束
	*/
	RobotEvent wait_for_motors();

	/*
	@brief 等待电机运动结束
	@param const int motor_ids 电机编号
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否运动结束
	*/
	RobotEvent wait_for_motors(const int motor_ids);

	/*
	@brief 等待电机运动结束
	@param size_t		size		需要等待的电机数量
	@param const int*	motor_ids	一组电机编号
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否运动结束
	*/
	RobotEvent wait_for_motors(size_t size, const int* motor_ids);

	/*
	@brief 机械臂关节轨迹运动
	@param size_t			size			路径点数量
	@param Pose*			points			一系列路径点
	@param float			velocity		机械臂运动最大速度(默认值为30.0)
	@param float			acceleration	机械臂运动最大加速度(默认值为150.0)
	@param float			interval		轨迹点插补间隔(默认值为0.04)
	@param Traj_Close_Mode	mode			轨迹闭环模式,TrajCloseMode_Closed表示轨迹闭环,TrajCloseMode_Open表示轨迹开环(默认值为TrajCloseMode_Open)
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否运动结束
	*/
	RobotEvent moveJ(size_t size, Pose* points, float velocity = 20.0, float acceleration = 150.0, float interval = 0.04, Traj_Close_Mode mode = TrajCloseMode_Open);

	/*
	@brief 机械臂末端多边形轨迹运动
	@param size_t			size			路径点数量
	@param Pose*			points			一系列路径点
	@param const double*	radius			对应的路径点交融半径
	@param float			velocity		机械臂运动最大速度(默认值为30.0)
	@param float			acceleration	机械臂运动最大加速度(默认值为150.0)
	@param float			interval		轨迹点插补间隔(默认值为0.04)
	@param Traj_Close_Mode	mode			轨迹闭环模式,TrajCloseMode_Closed表示轨迹闭环,TrajCloseMode_Open表示轨迹开环(默认值为TrajCloseMode_Open)
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否运动结束
	*/
	RobotEvent moveL(size_t size, Pose* points, const double* radius, float velocity = 30.0, float acceleration = 150.0, float interval = 0.04, Traj_Close_Mode mode = TrajCloseMode_Open);

	/*
	@brief 机械臂末端复杂轨迹运动
	@param size_t			size			路径点数量
	@param Pose*			points			一系列路径点
	@param float			velocity		机械臂运动最大速度(默认值为30.0)
	@param float			acceleration	机械臂运动最大加速度(默认值为150.0)
	@param float			interval		轨迹点插补间隔(默认值为0.04)
	@param Traj_Close_Mode	mode			轨迹闭环模式,TrajCloseMode_Closed表示轨迹闭环,TrajCloseMode_Open表示轨迹开环(默认值为TrajCloseMode_Open)
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否运动结束
	*/
	RobotEvent moveP(size_t size, Pose* points, float velocity = 30.0, float acceleration = 150.0, float interval = 0.04, Traj_Close_Mode mode = TrajCloseMode_Open);

	/*
	@brief 松开机械臂关节
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent release();

	/*
	@brief 松开机械臂关节
	@param const int motor_ids 电机编号
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent release(const int motor_ids);

	/*
	@brief 松开机械臂关节
	@param size_t		size		需要等待的电机数量
	@param const int*	motor_ids	一组电机编号
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent release(size_t size, const int* motor_ids);

	/*
	@brief 抱紧机械臂关节
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent hold();

	/*
	@brief 抱紧机械臂关节
	@param const int motor_ids 电机编号
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent hold(const int motor_ids);

	/*
	@brief 抱紧机械臂关节
	@param size_t		size		需要等待的电机数量
	@param const int*	motor_ids	一组电机编号
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent hold(size_t size, const int* motor_ids);

	/*
	@brief 停止机械臂运动并进入不可运动状态
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent stop();

	/*
	@brief 恢复到机械臂可运动状态
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent recover();

	/*
	@brief 校准机械臂
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent calibrate();

	/*
	@brief 复位机械臂
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent reset();

	/*
	@brief 获取电机编号与电机数量
	@param size_t		&size		电机数量参数返回
	@param int*			motor_ids	一组电机编号参数返回
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_motor_ids(size_t &size, int* motor_ids);

	/*
	@brief 获取关节角度
	@param const int	motor_ids	电机编号
	@param double		&angles		返回的角度值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_angles(const int motor_ids, double &angles);

	/*
	@brief 获取关节角度
	@param double*		angles		返回所有关节的角度值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_angles(double* angles);

	/*
	@brief 获取关节角度
	@param size_t		size		需要获取角度的电机数量
	@param const int*	motor_ids	一组电机编号
	@param double*		angles		电机编号对应的角度值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_angles(size_t size, const int* motor_ids, double* angles);

	/*
	@brief 获取关节速度
	@param const int	motor_ids	电机编号
	@param double		&velocity	返回的速度值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_velocity(const int motor_ids, double &velocity);

	/*
	@brief 获取关节速度
	@param double*		velocity		返回所有关节的速度值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_velocity(double* velocity);

	/*
	@brief 获取关节速度
	@param size_t		size		需要获取速度的电机数量
	@param const int*	motor_ids	一组电机编号
	@param double*		velocity	电机编号对应的速度值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_velocity(size_t size, const int* motor_ids, double* velocity);

	/*
	@brief 获取关节电流
	@param const int	motor_ids	电机编号
	@param double		&current	返回的电流值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_current(const int motor_ids, double &current);

	/*
	@brief 获取关节电流
	@param double*		current		返回所有关节的电流值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_current(double* current);

	/*
	@brief 获取关节电流
	@param size_t		size		需要获取电流的电机数量
	@param const int*	motor_ids	一组电机编号
	@param double*		current		电机编号对应的电流值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_current(size_t size, const int* motor_ids, double* current);

	/*
	@brief 获取末端位姿
	@param Pose			&pose		获取的末端位姿返回值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_pose(Pose &pose);

	/*
	@brief 前向运动学解算
	@param const double*	angles		与关节数量对应的一些列关节角度值
	@param Pose				&pose		解算获得的末端位姿值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent forward(const double* angles, Pose &pose);

	/*
	@brief 逆向运动学解算
	@param const Pose	pose		机械臂末端位姿值
	@param double*		angles		解算获得的关节角度值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent inverse(Pose pose, double* angles);

	/*
	@brief 获取关节io输入
	@param const int	motor_ids	电机编号
	@param int			&signal	返回的信号值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_input(const int motor_ids, int &signal);

	/*
	@brief 获取关节io输入
	@param int		&signal		返回所有关节的信号值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_input(int* signal);

	/*
	@brief 获取关节io输入
	@param size_t		size		需要获取信号的电机数量
	@param const int	motor_ids	一组电机编号
	@param int			&signal		电机编号对应的信号值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent get_input(size_t size, const int* motor_ids, int* signal);

	/*
	@brief 设置关节io输出
	@param const int	motor_ids	一组电机编号
	@param const int	signal		电机编号对应的信号值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent set_output(const int motor_ids, const int signal);

	/*
	@brief 设置关节io输出
	@param size_t		size		需要设置信号的电机数量
	@param const int	motor_ids	一组电机编号
	@param const int*	&signal		电机编号对应的信号值
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent set_output(size_t size, const int* motor_ids, const int* signal);

	/*
	@brief 设置末端工具动作
	@param const char*	name		末端工具名称
	@param const char*	action		动作名称
	@return RobotEvent 机械臂事件
	@step 生成任务
	@step 获取任务返回值
	@step 查验是否成功
	*/
	RobotEvent set_EOAT_action(const char* name, const char* action);

};







#endif

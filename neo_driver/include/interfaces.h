#ifndef _INTERFACES_H
#define _INTERFACES_H

#include <dlfcn.h>
#include "metaType.h"

typedef NEOStatus(*ServiceLoginType)(const char*, short int, const char*, unsigned int, int);
typedef NEOStatus(*ServiceLogoutType)();
typedef NEOStatus(*RobotSetupType)(const char*, unsigned int);
typedef NEOStatus(*RobotShutdownType)();

typedef NEOStatus(*CalibrateType)();
typedef NEOStatus(*ReleaseType)(size_t, const int*);
typedef NEOStatus(*HoldType)(size_t, const int*);
typedef NEOStatus(*StopType)();
typedef NEOStatus(*HaltType)();
typedef NEOStatus(*PauseType)();
typedef NEOStatus(*ResumeType)();
typedef NEOStatus(*RecoverType)();

typedef NEOStatus(*GetJointIdsType)(size_t*, int*);

typedef int(*GetInnerStatusType)();

typedef NEOStatus(*GetJointsType)(size_t, const int*, double*);
typedef NEOStatus(*GetEndposType)(Pose*);
typedef NEOStatus(*GetVelocityType)(size_t, const int*, double*);
typedef NEOStatus(*GetCurrentType)(size_t, const int*, double*);
typedef NEOStatus(*GetInputIOType)(size_t, const int*, int*);
typedef NEOStatus(*GetJointParametersType)(size_t, const int*, int, double*);
typedef NEOStatus(*GetDefaultParametersType)(const char*, double*);

typedef NEOStatus(*SetOutputIOType)(size_t, const int*, const int*);

typedef NEOStatus(*AddMonitorParamType)(int);
typedef NEOStatus(*RemoveMonitorParamType)(int);

typedef NEOStatus(*MoveJointsType)(size_t, const int*, const double*, const double, const double, unsigned int);
typedef NEOStatus(*MoveEndposType)(Pose, const double, const double, unsigned int);
typedef NEOStatus(*WaitForJointsType)(size_t, const int*);

typedef NEOStatus(*MoveJType)(size_t, DoubleArray*, const double, const double, const double, unsigned int, int);
typedef NEOStatus(*MoveLType)(size_t, Pose*, const double, const double, const double, unsigned int, int);
typedef NEOStatus(*MovePType)(size_t, Pose*, const double, const double, const double, unsigned int, int);

typedef NEOStatus(*ForwardType)(const double*, Pose*);
typedef NEOStatus(*InverseType)(Pose, const double*, DoubleArray*);

typedef NEOStatus(*LoadEndEffectorType)(const char* name);
typedef NEOStatus(*EmitEESignalType)(unsigned int);
typedef NEOStatus(*UnloadEndEffectorType)();

typedef NEOStatus(*LoadProjectType)(const char*);
typedef NEOStatus(*CreateProjectType)(const char*);
typedef NEOStatus(*DeleteProjectType)(const char*);
typedef NEOStatus(*CheckProjectType)(const char*, int*);
typedef NEOStatus(*SavePoseType)(const char*, const double*);
typedef NEOStatus(*DeletePoseType)(const char*);
typedef NEOStatus(*GetPoseType)(const char*, DoubleArray*);

typedef NEOStatus(*EnableCollisionProtectionType)(int);
typedef NEOStatus(*SetCollisionProtectionLevelType)(int);
typedef NEOStatus(*ForbidCollisionProtectionType)();

typedef NEOStatus(*EnableKinestheticTeachingType)(double);
typedef NEOStatus(*DisableKinestheticTeachingType)();

typedef NEOStatus(*EnableVelocityTuningType)(int);
typedef NEOStatus(*DisableVelocityTuningType)();
typedef NEOStatus(*SetJointVelocitiesType)(size_t, double*);
typedef NEOStatus(*SetEndposVelocitiesType)(double*);

typedef NEOStatus(*EnablePositionTuningType)();
typedef NEOStatus(*DisablePositionTuningType)();
typedef NEOStatus(*SetTargetType)(size_t, double*, double*);

typedef NEOStatus(*GetRotationMatrixType)(RotationMatrix*);

typedef NEOStatus(*SaveEEKinematicsType)(Pose);
typedef NEOStatus(*GetEEKinematicsType)(Pose*);
typedef NEOStatus(*SetEndposOffsetType)(Pose);


class Interfaces
{
private:
    const char* NeoSI_lib = "libNeoService4c.so";
    void* NeoSI = NULL;

public:
    Interfaces(){}
    ~Interfaces(){}

    bool load();
    bool unload();

    /**********************************************************************************************************************
    Function: 与机械臂服务器建立连接
    Input arguments:
    @param: const char*     ip          机械臂网络IP
    @param: short int       port        机械臂网络端口
    @param: const char*     serial      机械臂序列号
    @param: unsigned int    timeout     连接超时时间
    @param: int             transport   通讯方式
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus ServiceLogin(const char* ip, short int port, const char* serial, unsigned int timeout, int transport);

    /**********************************************************************************************************************
    Function: 断开与机械臂服务器连接
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus ServiceLogout();

    /**********************************************************************************************************************
    Function: 初始化机械臂
    Input arguments:
    @param: const char*     name    机械臂名称
    @param: unsigned int    mode    连接模式
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus RobotSetup(const char* name, unsigned int mode);

    /**********************************************************************************************************************
    Function: 关闭机械臂
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus RobotShutdown();

    /**********************************************************************************************************************
    Function: 校准机械臂关节
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Calibrate();

    /**********************************************************************************************************************
    Function: 松开机械臂关节
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids   松开的关节ID
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Release(size_t size, const int* joint_ids);

    /**********************************************************************************************************************
    Function: 抱紧机械臂关节
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids   松开的关节ID
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Hold(size_t size, const int* joint_ids);

    /**********************************************************************************************************************
    Function: 停止机械臂当前运动
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Stop();

    /**********************************************************************************************************************
    Function: 停止机械臂当前运动并进入不可运动模式
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Halt();

    /**********************************************************************************************************************
    Function: 暂停当前运动
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Pause();

    /**********************************************************************************************************************
    Function: 继续运动
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Resume();

    /**********************************************************************************************************************
    Function: 从不可运动模式中恢复
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Recover();

    /**********************************************************************************************************************
    Function: 获取机械臂关节ID
    Input arguments:
    @param: size_t*   size        机械臂关节数量
    @param: int*      joint_ids   机械臂关节ID号
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetJointIds(size_t* size, int* joint_ids);

    /**********************************************************************************************************************
    Function: 获取机械臂内部状态 
    Output arguments:
    @value: int
       ready     |             run(6-8)              |                 quiescence(1-5)                     |  emergency
	       9       |     8          7            6     |     5         4         3          2          1     |      0
     true/false  |  ST_NONE  ST_RUN_MOVE  ST_RUN_PVT |  ST_NONE   ST_PAUSE   ST_STOP   ST_CEASE   ST_HALT  | true/false
    **********************************************************************************************************************/
    int GetInnerStatus();

    /**********************************************************************************************************************
    Function: 获取机械臂关节角度
    Input arguments:
    @param: size_t        size        
    @param: const int*    joint_ids   机械臂关节ID号
    @param: double*       joints      获取到的关节角度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetJoints(size_t size, const int* joint_ids, double* joints);

    /**********************************************************************************************************************
    Function: 获取机械臂末端位姿
    Input arguments:
    @value  Pose*     pose    末端位姿
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetEndpos(Pose* pose);

    /**********************************************************************************************************************
    Function: 获取机械臂关节速度
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids   机械臂关节ID号
    @param: double*       velocity    获取到的关节速度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetVelocity(size_t size, const int* joint_ids, double* velocity);

    /**********************************************************************************************************************
    Function: 获取机械臂关节电流
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids   机械臂关节ID号
    @param: double*       current     获取到的关节速度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetCurrent(size_t size, const int* joint_ids, double* current);

    /**********************************************************************************************************************
    Function: 获取输入信号
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids   机械臂关节ID号
    @param: int*          signal      获取到的输入信号
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetInputIO(size_t size, const int* joint_ids, int* signal);

    /**********************************************************************************************************************
    Function: 获取机械臂关节相关参数
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids   机械臂关节ID号
    @param: const int     name        参数名称
    @param: double*       parameters  获取到的关节参数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetJointParameters(size_t size, const int* joint_ids, int name, double* parameters);

    /**********************************************************************************************************************
    Function: 获取机械臂运动默认参数
    Input arguments:
    @param: const char*   name        运动默认参数名称
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetDefaultParameters(const char* name, double* parameters);

    /**********************************************************************************************************************
    Function: 设置IO输出
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids   机械臂关节ID号
    @param: int*          signal      输出信号
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SetOutputIO(size_t size, const int* joint_ids, const int* signal);

    /**********************************************************************************************************************
    Function: 添加后台监控参数
    Input arguments:
    @param: int           status    监控参数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus AddMonitorParam(int status);

    /**********************************************************************************************************************
    Function: 移除后台监控参数
    Input arguments:
    @param: int           status    监控参数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus RemoveMonitorParam(int status);

    /**********************************************************************************************************************
    Function: 根据梯形速度曲线运动机械臂关节到指定关节位置
    Input arguments:
    @param: size_t          size
    @param: const int*      joint_ids       机械臂关节ID号
    @param: const double*   joints          目标关节位置
    @param: const double    velocity        运动最大速度
    @param: const double    acceleration    运动最大加速度
    @param: unsigned int    mode            运动模式
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus MoveJoints(size_t size, const int* joint_ids, const double* joints, const double velocity, const double acceleration, unsigned int mode);

    /**********************************************************************************************************************
    Function: 根据梯形速度曲线运动机械臂末端到指定位姿
    Input arguments:
    @param: Pose            pose            指定位姿
    @param: float           velocity        运动最大速度
    @param: float           acceleration    运动最大加速度
    @param: unsigned int    mode            运动模式
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus MoveEndpos(Pose pose, const double velocity, const double acceleration, unsigned int mode);

    /**********************************************************************************************************************
    Function: 等待梯形速度运动结束
    Input arguments:
    @param: size_t        size
    @param: const int*    joint_ids     机械臂关节ID号     
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus WaitForJoints(size_t size, const int* joint_ids);

    /**********************************************************************************************************************
    Function: 机械臂关节PVT运动
    Input arguments:
    @param: size_t          size
    @param: Pose*           points          关节路点
    @param: const double    velocity        运动最大速度
    @param: const double    acceleration    运动最大加速度
    @param: const double    interval        插补间隔
    @param: unsigned int    mode            运动模式
    @param: unsigned int    loop            循环次数

    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus MoveJ(size_t size, DoubleArray* points, const double velocity, const double acceleration, const double interval, unsigned int mode, int loop);

    /**********************************************************************************************************************
    Function: 机械臂末端多边形运动
    Input arguments:
    @param: size_t          size
    @param: Pose*           points          末端位姿路点
    @param: const double    velocity        运动最大速度
    @param: const double    acceleration    运动最大加速度
    @param: const double    interval        插补间隔
    @param: unsigned int    mode            运动模式
    @param: unsigned int    loop            循环次数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus MoveL(size_t size, Pose* points, const double velocity, const double acceleration, const double interval, unsigned int mode, int loop);

    /**********************************************************************************************************************
    Function: 机械臂末端曲线运动
    Input arguments:
    @param: size_t          size
    @param: Pose*           points          末端位姿路点
    @param: const double    velocity        运动最大速度
    @param: const double    acceleration    运动最大加速度
    @param: const double    interval        插补间隔
    @param: unsigned int    mode            运动模式
    @param: unsigned int    loop            循环次数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus MoveP(size_t size, Pose* points, const double velocity, const double acceleration, const double interval, unsigned int mode, int loop);

    /**********************************************************************************************************************
    Function: 正运动学解算
    Input arguments:
    @param: const double*   joitns    关节角度
    @param: Pose*           pose      正运动学解算的末端位姿
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Forward(const double* joints, Pose* pose);

    /**********************************************************************************************************************
    Function: 逆运动学解算
    Input arguments:
    @param: Pose            pose        末端位姿
    @param: const double*   previews    预期角度
    @param: DoubleArray*    joints      逆运动学解算的关节角度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus Inverse(Pose pose, const double* previews, DoubleArray* joints);

    /**********************************************************************************************************************
    Function: 加载末端工具
    Input arguments:
    @param: const char*   name    末端工具名称
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus LoadEndEffector(const char* name);

    /**********************************************************************************************************************
    Function: 发送末端工具控制信号
    Input arguments:
    @param: unsigned int    signal    控制信号
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus EmitEESignal(unsigned int signal);

    /**********************************************************************************************************************
    Function: 卸载末端工具
    Input arguments:
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus UnloadEndEffector();

    /**********************************************************************************************************************
    Function: 获取项目列表
    Input arguments:
    @param: size_t* size
    @param: string* list
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    // NEOStatus GetProjectList(size_t* size, string* list);


    /**********************************************************************************************************************
    Function: 加载项目
    Input arguments:
    @param: const char*   name    项目名称
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus LoadProject(const char* name);

    /**********************************************************************************************************************
    Function: 创建项目
    Input arguments:
    @param: const char*   name    项目名称
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus CreateProject(const char* name);

    /**********************************************************************************************************************
    Function: 删除项目
    Input arguments:
    @param: const char*   name    项目名称
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus DeleteProject(const char* name);

    /**********************************************************************************************************************
    Function: 检查项目是否存在
    Input arguments:
    @param: const char*   name    项目名称
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus CheckProject(const char* name, int* result);

    /**********************************************************************************************************************
    Function: 保存关节角度信息到项目中
    Input arguments:
    @param: const char*     name    动作名称
    @param: const double*   pose    关节角度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SavePose(const char* name, const double* pose);

    /**********************************************************************************************************************
    Function: 删除关节角度信息
    Input arguments:
    @param: const char*   name    动作名称
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus DeletePose(const char* name);

    /**********************************************************************************************************************
    Function: 获取对应名字的关节角度信息
    Input arguments:
    @param: const char*   name    动作名称
    @param: DoubleArray*  pose    关节角度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetPose(const char* name, DoubleArray* pose);


    /**********************************************************************************************************************
    Function: 获取动作名称列表
    Input arguments:
    @param: size_t*   size
    @param: string*   size
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    // NEOStatus GetPoseList(size_t* size, string* list);


    /**********************************************************************************************************************
    Function: 使能碰撞保护
    Input arguments:
    @value  int   level   碰撞保护等级
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus EnableCollisionProtection(int level);

    /**********************************************************************************************************************
    Function: 设置碰撞保护等级
    Input arguments:
    @value  int   level   碰撞保护等级
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SetCollisionProtectionLevel(int level);

    /**********************************************************************************************************************
    Function: 关闭碰撞保护
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus ForbidCollisionProtection();

    /**********************************************************************************************************************
    Function: 开启轻松拖拽示教功能
    Input arguments:
    @value double   load    末端负载
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus EnableKinestheticTeaching(double load);

    /**********************************************************************************************************************
    Function: 关闭轻松拖拽示教功能
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus DisableKinestheticTeaching();

    /**********************************************************************************************************************
    Function: 使能机械臂速度微调功能
    Input arguments:
    @value int   mode    速度微调模式
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus EnableVelocityTuning(int mode);

    /**********************************************************************************************************************
    Function: 关闭机械臂速度微调功能
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus DisableVelocityTuning();

    /**********************************************************************************************************************
    Function: 设置末端位姿微调速度
    Input arguments:
    @value  double*   endpos_velocities   末端速度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SetEndposVelocities(double* endpos_velocities);

    /**********************************************************************************************************************
    Function: 设置关节角度微调速度
    Input arguments:
    @value  size_t    size                
    @value  double*   joint_velocities    关节速度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SetJointVelocities(size_t size, double* joint_velocities);

    /**********************************************************************************************************************
    Function: 使能机械臂位置微调功能
    Output arguments:
    @value: NEOCStatus
    **********************************************************************************************************************/
    NEOStatus EnablePositionTuning();

    /**********************************************************************************************************************
    Function: 关闭机械臂位置微调功能
    Output arguments:
    @value: NEOCStatus
    **********************************************************************************************************************/
    NEOStatus DisablePositionTuning();

    /**********************************************************************************************************************
    Function: 设置机械臂关节微调目标位置
    Input arguments:
    @value  size_t    size          
    @value  double*   joints        关节角度
    @value  double*   velocities    关节速度
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SetTarget(size_t size, double* joints, double* velocities);

    /**********************************************************************************************************************
    Function: 获得末端旋转矩阵
    Input arguments:
    @value  RotationMatrix*   rotation_matrix     末端旋转矩阵 
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetRotationMatrix(RotationMatrix* rotation_matrix);

    /**********************************************************************************************************************
    Function: 保存末端工具运动学参数
    Input arguments:
    @value  Pose    pose    运动学参数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SaveEEKinematics(Pose pose);

    /**********************************************************************************************************************
    Function: 获取末端工具运动学参数
    Input arguments:
    @value  Pose*     pose    运动学参数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus GetEEKinematics(Pose* pose);

    /**********************************************************************************************************************
    Function: 设置末端偏移
    Input arguments:
    @value  Pose    pose    偏移参数
    Output arguments:
    @value: NEOStatus
    **********************************************************************************************************************/
    NEOStatus SetEndposOffset(Pose pose);
};

#endif

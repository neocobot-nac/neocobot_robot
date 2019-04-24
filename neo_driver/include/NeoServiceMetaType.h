#ifndef _NEOSERVICEMETATYPE_H
#define _NEOSERVICEMETATYPE_H

#ifndef _NEOCLIENT_BEGIN
#define _NEOCLIENT_BEGIN namespace NeoClient {
#endif

#ifndef NEOCLIENT
#define NEOCLIENT NeoClient
#endif

#ifndef _NEOCLIENT_END
#define _NEOCLIENT_END }
#endif


#define NEOCStatus unsigned long

#define NEOCLIENT_OK				0x00000000U // everything is ok(0)
/* client */
#define NEOCLIENT_ERROR_SYSE		0x00000001U // System-level errors(1)
#define NEOCLIENT_ERROR_SE			0x00000002U // Send error in Sender thread(2)
#define NEOCLIENT_ERROR_RE			0x00000004U // Receive error in Receiver thread(4)
#define NEOCLIENT_ERROR_GF			0x00000008U // generate msg error(8)
#define NEOCLIENT_ERROR_GRF			0x00000010U // Get result failed(16)
#define NEOCLIENT_ERROR_RT			0x00000020U // Receive timeout(32)
#define NEOCLIENT_ERROR_ISE			0x00000040U // Initialize socket error(64)
#define NEOCLIENT_ERROR_UF			0x00000080U // Unwrap msg error(128)
/* server */
#define NEOCLIENT_ERROR_FNF			0x00000100U // Function Not Found(256)
#define NEOCLIENT_ERROR_VM			0x00000200U // Function cannot be used in this version(512)
#define NEOCLIENT_ERROR_UA			0x00000400U // Unexpected arguments(1024)
#define NEOCLIENT_ERROR_RNI			0x00000800U // Robot not initialized(2048)
#define NEOCLIENT_ERROR_RAI			0x00001000U // Robot already initialized(4096)
#define NEOCLIENT_ERROR_REE			0x00002000U // Robot execute error(8192)
#define NEOCLIENT_ERROR_NRP			0x00004000U // Register params is nonstandrad(16384)
#define NEOCLIENT_ERROR_RGE			0x00008000U // Register error(32768)
#define NEOCLIENT_ERROR_NR			0x00010000U // Client not register(65536)
#define NEOCLIENT_ERROR_NIP			0x00020000U // ClientInfo is nonstandrad(131072)
#define NEOCLIENT_ERROR_NC			0x00040000U // Not connected yet(262144)
#define NEOCLIENT_ERROR_PF			0x00080000U // The Conn msg does not have "head" element(524288)

#define NEOCLIENT_TERMINATOR		0x01000000U // for special use instead of errorcode(16777216)
#define NEOCLIENT_ERROR_UNKNOWN		0x10000000U // unknown error(268435456)

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

#endif

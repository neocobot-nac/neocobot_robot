#ifndef _NEOCOBOT_H
#define _NEOCOBOT_H

#if defined(WINDOWS) || defined(WIN32)
#else
#include <unistd.h>
#endif

#include <string.h>
#include <system_error>

using std::string;

/* Sleep (unit millisecond) */
#if defined(WINDOWS) || defined(WIN32)
#define SLEEP(millisecond) Sleep(millisecond)
#else
#define SLEEP(millisecond) usleep(millisecond*1000)
#endif

#if defined(WINDOWS) || defined(WIN32)
#include <direct.h>
const string CURRENT_PATH = _getcwd(NULL, 0);
const string PATH_OPERATOR = "\\";
#else
#include <unistd.h>
const string CURRENT_PATH = getcwd(NULL, 0);
const string PATH_OPERATOR = "/";
#endif

#if defined(WINDOWS) || defined(WIN32)
#define NEOStatus unsigned long long
#else
#define NEOStatus unsigned long
#endif

#define NEOCOBOT_OK                 0x00000000U // everything is ok(0)
#define NEOCOBOT_ERROR_ILLJCFG      0x00000001U // illegal joint configuration(1)
#define NEOCOBOT_ERROR_ILLMCFG		  0x00000002U // illegal motor configuration(2)
#define NEOCOBOT_ERROR_ILLRCFG      0x00000004U // illegal robot configuration(4)
#define NEOCOBOT_ERROR_ILLEECFG     0x00000008U // illegal end effector configuration(8)
#define NEOCOBOT_ERROR_ILLFMT       0x00000010U // illegal format(16)
#define NEOCOBOT_ERROR_ILLVAL       0x00000020U // illegal value(32)
#define NEOCOBOT_ERROR_ILLTASK      0x00000040U // illegal task command(64)
#define NEOCOBOT_ERROR_INIT         0x00000080U // can not complete an initial procedure(128)
#define NEOCOBOT_ERROR_SYSENV	      0x00000100U // problem in system enveronment(256)
#define NEOCOBOT_ERROR_SYSCMD       0x00000200U // problem in system command(512)
#define NEOCOBOT_ERROR_TMLCMD       0x00000400U // problem in TML library(1024)
#define NEOCOBOT_ERROR_ERRACT       0x00000800U // can not perform action(2048)
#define NEOCOBOT_ERROR_NOFILE	      0x00001000U // file doesn't exist(4096)
#define NEOCOBOT_ERROR_ILLFUNC      0x00002000U // illegal function(8192)
#define NEOCOBOT_ERROR_FROZEN       0x00004000U // controlboard is not actived(16384)
#define NEOCOBOT_ERROR_MSTATE       0x00008000U // switch motor into wrong state(32768)
#define NEOCOBOT_ERROR_POWER        0x00010000U // can not power motor(65536)
#define NEOCOBOT_ERROR_EXT          0x00020000U // extension error(131072)
#define NEOCOBOT_ERROR_CONFLICT     0x00040000U // Motion status conflict(262144)
#define NEOCOBOT_ERROR_UNKNOWN      0x00100000U // unknown error(1048576)

#define NEOCOBOT_TERMINATOR         0x01000000U // for special use instead of errorcode

#define NEOCOBOT_ERROR_JCFGL        (NEOCOBOT_ERROR_ILLJCFG | NEOCOBOT_ERROR_NOFILE)   // missing axis config file(4097)
#define NEOCOBOT_ERROR_JCFGF		    (NEOCOBOT_ERROR_ILLJCFG | NEOCOBOT_ERROR_ILLFMT)   // axis config lacks integrity(17)
#define NEOCOBOT_ERROR_JCFGV		    (NEOCOBOT_ERROR_ILLJCFG | NEOCOBOT_ERROR_ILLVAL)   // axis config maintains illegal value(33)

#define NEOCOBOT_ERROR_MCFGL        (NEOCOBOT_ERROR_ILLMCFG | NEOCOBOT_ERROR_NOFILE)   // missing motor config file(4098)
#define NEOCOBOT_ERROR_MCFGF		    (NEOCOBOT_ERROR_ILLMCFG | NEOCOBOT_ERROR_ILLFMT)   // motor config lacks integrity(18)
#define NEOCOBOT_ERROR_MCFGV		    (NEOCOBOT_ERROR_ILLMCFG | NEOCOBOT_ERROR_ILLVAL)   // motor config maintains illegal value(34)

#define NEOCOBOT_ERROR_CREATEFILE   (NEOCOBOT_ERROR_SYSCMD  | NEOCOBOT_ERROR_NOFILE)   // can't create folder or file(4608)

#define NEOCOBOT_ERROR_FUNCPARAM    (NEOCOBOT_ERROR_ILLFUNC | NEOCOBOT_ERROR_ILLFMT)   // missing parameters in function(8208)
#define NEOCOBOT_ERROR_FUNCPVAL     (NEOCOBOT_ERROR_ILLFUNC | NEOCOBOT_ERROR_ILLVAL)   // illegal value in function(8224)


/*  Transport type */
#define NEOMQTT 0
#define NEOSOCKET 1

/* Robot setup mode. */
#define OFFLINE 0
#define ONLINE 1

/* Max joints supported */
#define MAX_JOINTS 7

/* Max len of pose name supported */
#define MAX_POSE_LEN 256
#define MAX_PROJECT_LEN	256

/* Robot motion mode */
#define ABSOLUTE_MODE 0
#define RELATIVE_MODE 1

/* Robot trajectory motion mode */
#define OPEN_MODE 0
#define CLOSE_MODE 1

/* Robot velocity tuning mode */
#define JOINT_TUNING_MODE 0
#define ENDPOS_TUNING_MODE 1

/* Monitor status */
#define RD_POSITION 0
#define RD_CURRENT 1
#define RD_VELOCITY 0

#define RI_FAULT_STATUS 10
#define RI_BUFFER_STATUS 11
#define RI_EVENT_STATUS 12
#define RI_IO_SIGNAL 13

/* Default parameter name */
#define DEFAULT_VELOCITY_DEG "velocity_in_deg"
#define DEFAULT_ACCELERATION_DEG "acceleration_in_deg"
#define DEFAULT_VELOCITY_MM "velocity_in_mm"
#define DEFAULT_ACCELERATION_MM "acceleration_in_mm"
#define DEFAULT_INTERVAL_RED "interval_in_record"
#define DEFAULT_INTERVAL_CALC "interval_in_calculation"

/* Joint param name */
#define LOW_ANGLE_LIMIT 1
#define UPPER_ANGLE_LIMIT 2
#define VELOCITY_LIMIT 3
#define ACCELERATION_LIMIT 4
#define CURRENT_LIMIT 5
#define EMERGENCY_CURRENT 6
#define MOUTING_DIRECTION 7
#define MOTION_COMPLETE_LIMIT 8

/* Inner status */
#define ST_EMERGENCY 1
#define ST_HALT 2
#define ST_CEASE 4
#define ST_STOP 8
#define ST_PAUSE 16
#define ST_NONE 32
#define ST_RUN_PVT 64
#define ST_RUN_MOVE 128
#define ST_RUN_NONE 256
#define ST_READY 512

/* End position of robot */
typedef struct _Position
{
	double x;
	double y;
	double z;

	_Position(double _x, double _y, double _z) :x(_x), y(_y), z(_z) {};
	_Position() {};
}Position, * PPosition;

/* End orientation of robot */
typedef struct _Orientation
{
	double Rx;
	double Ry;
	double Rz;

	_Orientation(double _Rx, double _Ry, double _Rz) :Rx(_Rx), Ry(_Ry), Rz(_Rz) {};
	_Orientation() {};
}Orientation, * POrientation;

/* End Pose of robot */
typedef struct _Pose
{
	Position position;
	Orientation orientation;

	_Pose(Position _position, Orientation _orientation) :position(_position), orientation(_orientation) {};
	_Pose() {};
}Pose, * PPose;


typedef struct _DoubleArray
{
	size_t  len;
	double  value[MAX_JOINTS];

	_DoubleArray() {};
}DoubleArray, * PDoubleArray;

typedef struct _RotationMatrix
{
	size_t  len;
	double  value[16];

	_RotationMatrix() {};
}RotationMatrix, * PRotationMatrix;

#endif













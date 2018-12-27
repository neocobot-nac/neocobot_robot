#ifndef STDAFX_H
#define STDAFX_H

#ifdef _WIN32
	#include "atlbase.h"
	#include "atlstr.h"
	#include <windows.h>
	#include "logger.h"

#else
	#include <unistd.h>
	#include "logger.h"

	typedef unsigned long DWORD;

#endif

extern int Faststamp;
extern int Slowstamp;

int GetFastTimeStamp();
int GetSlowTimeStamp();

void timesleep(unsigned long nsec);

typedef int RobotEvent;

typedef struct SetupParam
{
	const char* channel_name = "/dev/pcanusb32";
	const char* channel_type = "PEAK_SYS_PCAN_USB";
	const char* protocol = "TMLCAN";
	int host_id = 10;
	int baud_rate = 500000;

}SETUPPARAM;

typedef enum
{
	REAL_MODE = 0,
	VIRTUAL_MODE = 1

}SETUPMODE;

typedef enum
{
	SUCCEED = 0,
	NORMALERROR = 1,
	RECVTIMEOUT = 2

}ROBOTEVENTTYPE;

typedef enum
{
	MOVE_NO_RELATIVE = 0,
	MOVE_RELATIVE = 1

}RELATIVEMODE;

typedef enum
{
	TRAJ_NO_CLOSED = 0,
	TRAJ_CLOSED = 1

}TRAJMODE;

typedef struct Pos
{
	double x;
	double y;
	double z;

}POS;

typedef struct Ori
{
	double roll;
	double pitch;
	double yaw;

}ORI;

typedef struct Pose
{
	POS position;
	ORI orientation;

}POSE;

#endif

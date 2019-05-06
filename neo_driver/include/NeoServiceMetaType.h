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
#define NEOCLIENT_ERROR_NL			0x00000100U // No login service(256)
#define NEOCLIENT_ERROR_NS			0x00000200U // No setup Robot(512)
#define NEOCLIENT_ERROR_TO			0x00000400U // Heart beat timeout(1024)
/* server */
#define NEOCLIENT_ERROR_FNF			0x00001000U // Function Not Found(4096)
#define NEOCLIENT_ERROR_VM			0x00002000U // Function cannot be used in this version(8192)
#define NEOCLIENT_ERROR_UA			0x00004000U // Unexpected arguments(16384)
#define NEOCLIENT_ERROR_RNI			0x00008000U // Robot not initialized(32768)
#define NEOCLIENT_ERROR_RAI			0x00010000U // Robot already initialized(65536)
#define NEOCLIENT_ERROR_REE			0x00020000U // Robot execute error(131072)
#define NEOCLIENT_ERROR_NRP			0x00040000U // Register params is nonstandrad(262144)
#define NEOCLIENT_ERROR_RGE			0x00080000U // Register error(524288)
#define NEOCLIENT_ERROR_NR			0x00100000U // Client not register(1048576)
#define NEOCLIENT_ERROR_NIP			0x00200000U // ClientInfo is nonstandrad(2097152)
#define NEOCLIENT_ERROR_NC			0x00400000U // Not connected yet(4194304)
#define NEOCLIENT_ERROR_PF			0x00800000U // The Conn msg does not have "head" element(8388608)

#define NEOCLIENT_TERMINATOR		0x10000000U // for special use instead of errorcode(268435456)
#define NEOCLIENT_ERROR_UNKNOWN		0x80000000U // unknown error(2147483648)

/* Robot setup mode. */
#define OFFLINE 1
#define ONLINE  0

/* Max joints supported */
#define MAX_JOINTS 10

/* Robot motion mode */
enum MotionMode :unsigned int
{
	ABSOLUTE_MODE = 10,
	RELATIVE_MODE,
};

/*  Robot trajectory motion mode */
enum TrajMotionMode :unsigned int
{
	CLOSED_MODE = 20,
	OPEN_MODE,
};

/* robot state */
typedef struct _RobotState
{
    bool Simulator;
    bool CANBusState;
    bool PowerOn;
    bool ForceControl;
    bool EmergencyStop;
    bool Release;
    bool InMotion;
    bool ReadyToMotion;
}RobotState, *PRobotState;

/* End position of robot */
typedef struct _Position
{
	double x;
	double y;
	double z;

	_Position(double _x, double _y, double _z) :x(_x), y(_y), z(_z) {};
	_Position() {};
}Position, *PPosition;

/* End orientation of robot */
typedef struct _Orientation
{
	double Rx;
	double Ry;
	double Rz;

	_Orientation(double _Rx, double _Ry, double _Rz) :Rx(_Rx), Ry(_Ry), Rz(_Rz) {};
	_Orientation() {};
}Orientation, *POrientation;

/* End Pose of robot */
typedef struct _Pose
{
	Position position;
	Orientation orientation;

	_Pose(Position _position, Orientation _orientation) :position(_position), orientation(_orientation) {};
	_Pose() {};
}Pose, *PPose;

#endif

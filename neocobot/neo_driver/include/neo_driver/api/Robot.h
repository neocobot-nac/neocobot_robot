#ifndef ROBOT_H
#define ROBOT_H

#if defined(__WIN32__) || defined(WIN32)
#define NEOAPI_DLLSHARED_EXPORT __declspec(dllexport)
#else
#define NEOAPI_DLLSHARED_EXPORT
#endif

#include "stdafx.h"
#include "sendthread.h"
#include "recvthread.h"
#include "heart.h"
#include "tasks.h"

#ifdef _WIN32
	#include <Winsock2.h>
	#pragma comment(lib, "ws2_32.lib")
#else
	#include <sys/socket.h>
	#include <netinet/in.h>
	#include <arpa/inet.h>
	#include <algorithm>
	#include <sstream>
	#include <string>
#endif

using namespace std;

class NEOAPI_DLLSHARED_EXPORT Robot
{
private:
	MyLogger * myLoger = NULL;

	SendThread send;
	RecvThread recv;
	HeartThread heart;

	#ifdef _WIN32
		WSADATA wsa;
		struct sockaddr_in si_other;
		int slen = sizeof(si_other);
		SOCKET s;
	#else
		struct sockaddr_in si_other;
		unsigned int slen = sizeof(si_other);
		int s;
	#endif

	bool isLogin = false;
	bool isSetup = false;

	RobotEvent __InitializeSocket(const char* ServerIp, short int ServerPort);
	RobotEvent __FinalizeSocket();

	int __FastGetResult(string name, map<string, string> &data);
	int __SlowGetResult(string name, map<string, string> &data);
	char* __GetClientIP();
	char* __GetSetupParam(SETUPPARAM param);
	char* __GetMotorIds(vector<char> motor_ids);
	char* __GetAngles(vector<double> angles);
	char* __GetPosition(POSE position);
	char* __GetPointsA(vector<vector<double>> points);
	char* __GetPointsP(vector<POSE> points);
	char* __GetRadius(vector<double> radius);
	char* __GetSignal(vector<char> motor_ids, vector<int> signal);
	void __SplitString(const string& s, vector<string>& v, const string& c);

public:
	Robot();
	~Robot();

	RobotEvent RobotLogin(const char* ServerIp, short int ServerPort);
	RobotEvent RobotLogout();

	RobotEvent Setup(const char* name, SETUPPARAM parameters, int mode);
	RobotEvent Shutdown();

	RobotEvent move_to_angles(vector<char> motor_ids, vector<double> angles, float velocity = 30.0, float acceleration = 150.0, int relative = MOVE_NO_RELATIVE);
	RobotEvent move_to_position(POSE position, float velocity = 30.0, float acceleration = 150.0, int relative = MOVE_NO_RELATIVE);
	RobotEvent wait_for_motors(vector<char> motor_ids);
	RobotEvent moveJ(vector<vector<double> > points, float velocity = 20.0, float acceleration = 150.0, float interval = 0.02, int closed = TRAJ_NO_CLOSED);
	RobotEvent moveL(vector<POSE> points, vector<double> radius, float velocity = 30.0, float acceleration = 150.0, float interval = 0.05, int closed = TRAJ_NO_CLOSED);
	RobotEvent moveP(vector<POSE> points, float velocity = 30.0, float acceleration = 150.0, float interval = 0.02, int closed = TRAJ_NO_CLOSED);
	RobotEvent stop();
	RobotEvent recover();
	RobotEvent release(vector<char> motor_ids);
	RobotEvent hold(vector<char> motor_ids);
	RobotEvent calibrate();
	RobotEvent reset();
	RobotEvent get_motor_ids(vector<char> &motor_ids);
	RobotEvent get_angles(vector<char> motor_ids, vector<double> &angles);
	RobotEvent get_position(POSE &position);
	RobotEvent get_velocity(vector<char> motor_ids, vector<double> &velocity);
	RobotEvent get_current(vector<char> motor_ids, vector<double> &current);

	RobotEvent forward(vector<double> angles, POSE &position);
	RobotEvent inverse(POSE position, vector<double> &angles);

	RobotEvent get_input(vector<char> motor_ids, vector<int> &signal);
	RobotEvent set_output(vector<char> motor_ids, vector<int> signal);

	RobotEvent auto_scan_path(vector<vector<POSE>> &path, vector<POSE> points, float increment = 20.0, float cus_increment = 30.0, int cus_num = 0);
};

#endif
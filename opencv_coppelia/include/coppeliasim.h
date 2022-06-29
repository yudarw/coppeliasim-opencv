#pragma once
#include <stdlib.h>
#include <iostream>
#include <windows.h>
#include <stdio.h>
#include <process.h>
#include <fstream>
//#include <msclr\marshal_cppstd.h>

using namespace std;

extern "C" {
#include "extApi.h"
}

using namespace std;

#define RAD_TO_DEG (180/3.141592)
#define DEG_TO_RAD (3.141592/180)


// ===================================== //
//			Class Coppeliasim
// ===================================== //
class CoppeliaSim {
public:
	bool connected;
	int clientID;
	int max_object;

public:
	bool connect(int port);
	void disconnect(int clientID);
	void startSimulation();
	void stopSimulation();
	void addObject(int obj_handle, float pos[3]);
	void removeObject(int obj_handle);
};



// ===================================== //
//		Class Robot for robot arm
// ===================================== //
class CoppeliaRobot: public CoppeliaSim{
private:
	int		ikTargetHandle;
	int		ikTipHandle;
	int		robotHandle;
	int		targetPosHandle;
	string	scriptName;
	string	robot_name;
	int		moving;
	int		__cnt;
	float* __position;

public:
	CoppeliaRobot(string name);
	int init();
	// Set cartisian position
	void setPosition(float pos[6], bool wait);
	void setPosition(float x, float y, float z, float w, float p, float r, bool wait);
	void setPosition(float x, float y, float z, float w, float p, float r);
	// Set joint position
	void setJointPosition(float joint[6], bool wait);
	void setJointPosition(float J1, float J2, float J3, float J4, float J5, float J6);
	// Read position
	void readPosition(float pos[6]);
	void readJointPosition(float joint_pos[6]);
	void setPosition2(float pos[6]);				// Direct set object position (no speed control)
	void readObjectPosition(string object_name, float current_pos[6]);
	bool isMoving();
	void setSpeed(int velocity);
	void gripperCatch();
	void gripperRelease();
	void getObjectMatrix(float M[4][4]);
};


enum { omniplatform, pioneer, kuka_youbot };
enum { vision_sensor, force_sensor, proximity_sensor };

// ============================================== //
//			Class for mobile robot
// ============================================== //
class CoppeliaRobotMobile : public CoppeliaSim {
private:
	int robot_type;
	int robot_handle;
	int motor_handle[4];
	string robot_name;

public:
	CoppeliaRobotMobile(int type, string name);
	int init();
	void move(float v_left, float v_right);
};


// ============================================== //
//					Class sensor 
// ============================================== //
typedef struct prox_t {
	simxUChar state;
	float objPoint[3];
	float objNorm[3];
	int objHandle;
};

class CoppeliaSensor : public CoppeliaSim {
private:
	int sensor_handle;
	int sensor_type;
	string sensor_name;
	simxUChar* image;
	int resolution[2];

public:
	prox_t data;

public:
	CoppeliaSensor(int type, string name);
	int		init();
	void	get_image(simxUChar ** img, int res[2]);
	int		get_state();
	void	read_force(float data_force[6]);
};


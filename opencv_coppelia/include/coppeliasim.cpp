#include "coppeliasim.h"
#include <windows.h>

// Connect to coppelia sim, defaulf port: 19997
/////////////////////////////////////////////////////////////////////
bool CoppeliaSim::connect(int port)
{
	if (!connected) {
		simxFinish(-1);
		this->clientID = simxStart((simxChar*)"127.0.0.1", port, true, true, 2000, 5);
		if (this->clientID != -1) {
			this->connected = true;
			printf("> CoppeliaSIM connection success! Client ID : %d \n", clientID);
			// Start simulation:
			//simxStartSimulation(clientID, simx_opmode_oneshot_wait);
			return true;
		}
		else {
			printf("> Simxstart failed!!! \n");
		}
	}
	else {
		printf("> CoppeliaSim already connected. Please disconnect first. \n");
	}
	return false;
}


// Diconnect from CoppeliaSim
////////////////////////////////////////////////////////////////////
void CoppeliaSim::disconnect(int clientID)
{
	this->connected = false;
	simxFinish(clientID);
}
// Start Simulation
void CoppeliaSim::startSimulation() {
	simxStartSimulation(clientID, simx_opmode_blocking);
}

// Stop simulation
void CoppeliaSim::stopSimulation() {
	simxStopSimulation(clientID, simx_opmode_blocking);
}

// Coppeliasim Add Object
void CoppeliaSim::addObject(int handle, float pos[3]) {
	int* newObjHandle;
	int objCnt;
	simxCopyPasteObjects(clientID, &handle, 1, &newObjHandle, &objCnt, simx_opmode_blocking);
	simxSetObjectPosition(clientID, newObjHandle[0], -1, pos, simx_opmode_blocking);
	simxSetObjectInt32Param(clientID, newObjHandle[0], sim_shapeintparam_static, 0, simx_opmode_blocking);
}

// Coppeliasim 
void CoppeliaSim::removeObject(int handle){
	simxRemoveObject(clientID, handle, simx_opmode_blocking);
}




// ==================================================================
//					COPPELIASIM ROBOT CLASS
// ==================================================================

CoppeliaRobot::CoppeliaRobot(string names) {
	robot_name = names;
}

// Robot initialization: retreive some handles:
//////////////////////////////////////////////////////////////////////	 
int CoppeliaRobot::init() {
	
	scriptName = robot_name;	// Name of the robot child script program
	string ik_target_name(robot_name + "/ikTarget");
	string ik_tip_name(robot_name + "/ikTip");

	// Get robot handles:
	simxGetObjectHandle(clientID, robot_name.c_str(), &robotHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, ik_target_name.c_str(), &ikTargetHandle, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, ik_tip_name.c_str(), &ikTipHandle, simx_opmode_oneshot_wait);

	cout << endl;
	cout << ">>> Initialize arm robot properies --> " << robot_name.c_str() << endl;
	cout << "Robot handle    = " << robotHandle << endl;
	cout << "ikTarget handle = " << ikTargetHandle << endl;
	cout << "ikTip handle	 = " << ikTipHandle << endl;
	cout << "====================================================== " << endl << endl;

	// -- Start some streaming data --
	// Get signal of robot moving status 
	simxGetIntegerSignal(clientID, "moving_status", &moving, simx_opmode_streaming);
	// Get updates of robot positon
	simxCallScriptFunction(
		clientID,
		scriptName.c_str(),					// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_getPosition",			// Function name
		0, NULL,							// inIntCnt, inInt
		0, NULL,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		&__cnt, &__position,				// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_streaming);
	
	// Get update of robot joint position
	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),						// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_getJointPosition",		// Function name
		0, NULL,							// inIntCnt, inInt
		0, NULL,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		&__cnt, &__position,					// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_streaming);

	return robotHandle;
}


// Set robot arm position:
// ////////////////////////////////////////////////////////////////////
void CoppeliaRobot::setPosition(float pos[6], bool wait) 
{

	setPosition(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);

	if (wait){
		while (isMoving());
	}
}
void CoppeliaRobot::setPosition(float x, float y, float z, float w, float p, float r, bool wait)
{
	setPosition(x, y, z, w, p, r);
	if (wait) {
		while (isMoving());
	}
}
void CoppeliaRobot::setPosition(float x, float y, float z, float w, float p, float r)
{
	float pos[6];
	pos[0] = x / 1000;			// meter
	pos[1] = y / 1000;			// meter
	pos[2] = z / 1000;			// meter
	pos[3] = w * DEG_TO_RAD;	// rad
	pos[4] = p * DEG_TO_RAD;	// rad
	pos[5] = r * DEG_TO_RAD;	// rad

	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),					// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_movePosition",			// Function name
		0, NULL,							// inIntCnt, inInt
		6, pos,								// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		NULL, NULL,							// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_oneshot);
}


// Set Robot Joint Position
///////////////////////////////////////////////////////////////////////////
void CoppeliaRobot::setJointPosition(float joint[6], bool wait) {
	setJointPosition(joint[0], joint[1], joint[2], joint[3], joint[4], joint[5]);
	if (wait) while (isMoving()); 
}
void CoppeliaRobot::setJointPosition(float J1, float J2, float J3, float J4, float J5, float J6) 
{
	float joint[6];
	joint[0] = J1 * (DEG_TO_RAD);
	joint[1] = J2 * (DEG_TO_RAD);
	joint[2] = J3 * (DEG_TO_RAD);
	joint[3] = J4 * (DEG_TO_RAD);
	joint[4] = J5 * (DEG_TO_RAD);
	joint[5] = J6 * (DEG_TO_RAD);

	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),					// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_moveJointPosition",		// Function name
		0, NULL,							// inIntCnt, inInt
		6, joint,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		NULL, NULL,							// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_oneshot);
}

// Read Current Robot Tip Position
////////////////////////////////////////////////////////////////////////////////////////
void CoppeliaRobot::readPosition(float currentPos[6])
{
	int res = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),					// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_getPosition",			// Function name
		0, NULL,							// inIntCnt, inInt
		0, NULL,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		&__cnt, &__position,				// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_buffer);

	if (res == simx_return_ok) {
		if (__position != nullptr) {
			currentPos[0] = __position[0] * 1000; //mm
			currentPos[1] = __position[1] * 1000; //mm
			currentPos[2] = __position[2] * 1000; //mm
			currentPos[3] = __position[3] * RAD_TO_DEG;
			currentPos[4] = __position[4] * RAD_TO_DEG;
			currentPos[5] = __position[5] * RAD_TO_DEG;
		}
	}
}


// Read Current Robot Joint Position
///////////////////////////////////////////////////////////////////////////////////////////
void CoppeliaRobot::readJointPosition(float joint_pos[6])
{
	int cnt;
	float* jointData;
	int* inData;
	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),						// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_getJointPosition",		// Function name
		0, NULL,							// inIntCnt, inInt
		0, NULL,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,						// outIntCnt, outInt
		&cnt, &jointData,					// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_buffer);

	if (result == simx_return_ok) {
		for (int i = 0; i < 6; i++) {
			joint_pos[i] = jointData[i] * RAD_TO_DEG;
		}
	}
}


// Get robot moving status. Return TRUE if the robot is moving and
// return FALSE is robot is stop.
bool CoppeliaRobot::isMoving() {
	simxGetIntegerSignal(clientID, "moving_status", &moving, simx_opmode_buffer);
	if (moving) return true;
	else return false;
}


// Arm Robot Gripper Catch
void CoppeliaRobot::gripperCatch()
{
	int state = 0;
	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),						// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_setGripper",						// Function name
		1, &state,							// inIntCnt, inInt
		0, NULL,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		NULL, NULL,							// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_oneshot);
}

// Arm Robot Gripper Gripper Release
void CoppeliaRobot::gripperRelease()
{
	int state = 1;
	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),						// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_setGripper",						// Function name
		1, &state,							// inIntCnt, inInt
		0, NULL,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		NULL, NULL,							// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_oneshot);
}

// Get Robot Tip Homogeneous Matrix:
/////////////////////////////////////////////////////////////////////////////////
void CoppeliaRobot::getObjectMatrix(float M[4][4])
{
	int cnt;
	float *pData;
	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),						// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"getObjectMatrix",						// Function name
		0, NULL,							// inIntCnt, inInt
		0, NULL,							// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		&cnt, &pData,						// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_oneshot_wait);

	M[0][0] = pData[0];
	M[0][1] = pData[1];
	M[0][2] = pData[2];
	M[0][3] = pData[3];

	M[1][0] = pData[4];
	M[1][1] = pData[5];
	M[1][2] = pData[6];
	M[1][3] = pData[7];

	M[2][0] = pData[8];
	M[2][1] = pData[9];
	M[2][2] = pData[10];
	M[2][3] = pData[11];

	M[3][0] = 0;
	M[3][1] = 0;
	M[3][2] = 0;
	M[3][3] = 1;
}


void CoppeliaRobot::readObjectPosition(string object_name, float currentPos[6]) {
	int object_handle;
	float position[3], orientation[3];
	simxGetObjectHandle(clientID, object_name.c_str(), &object_handle, simx_opmode_oneshot_wait);
	simxGetObjectPosition(clientID, object_handle, robotHandle, position, simx_opmode_oneshot_wait);
	simxGetObjectOrientation(clientID, object_handle, robotHandle, orientation, simx_opmode_oneshot_wait);
	currentPos[0] = position[0] * 1000;	//mm
	currentPos[1] = position[1] * 1000; //mm
	currentPos[2] = position[2] * 1000; //mm
	currentPos[3] = orientation[0] * RAD_TO_DEG;
	currentPos[4] = orientation[1] * RAD_TO_DEG;
	currentPos[5] = orientation[2] * RAD_TO_DEG;
}

// Directly set the robot position and orientation (no speed control)
void CoppeliaRobot::setPosition2(float pos[6])
{
	float position[3], orientation[3];
	for (int i = 0; i < 3; i++) {
		position[i] = pos[i] / 1000;	// mm
		orientation[i] = pos[i + 3] * DEG_TO_RAD;
	}
	simxSetObjectPosition(clientID, ikTargetHandle, robotHandle, position, simx_opmode_oneshot_wait);
	simxSetObjectOrientation(clientID, ikTargetHandle, robotHandle, orientation, simx_opmode_oneshot_wait);
}

// Set Robot Speed : 0 - 100
///////////////////////////////////////////////////////////////////////
void CoppeliaRobot::setSpeed(int velocity)
{
	float lin_vel = (float) velocity / 1000;
	float ang_vel = 10.0 * DEG_TO_RAD;

	float vel_data[2] = {lin_vel, ang_vel};

	int result = simxCallScriptFunction(
		clientID,
		scriptName.c_str(),						// the name of the associated obejct script
		sim_scripttype_childscript,			// the handle of the script
		"remoteApi_setSpeed",							// Function name
		0, NULL,							// inIntCnt, inInt
		2, vel_data,						// inFloatCnt, inFloat
		0, NULL,							// inStringCnt, inString
		0, NULL,							// inBufferSize, inBuffer
		NULL, NULL,							// outIntCnt, outInt
		NULL, NULL,							// outFloatCnt, outFloat
		NULL, NULL,							// outStringCnt, outString
		NULL, NULL,							// outBufferSize, outBuffer
		simx_opmode_blocking);
}





//////////////////////////////////////////////////////////////////
//					Coppelia Mobile Robot						//
//////////////////////////////////////////////////////////////////

CoppeliaRobotMobile::CoppeliaRobotMobile(int type, string name) {
	robot_name = name;
	robot_type = type;
}

int CoppeliaRobotMobile::init() {
	char str[128];

	simxGetObjectHandle(clientID, robot_name.c_str(), &robot_handle, simx_opmode_blocking);
	printf("Mobile robot handle: %d \n", robot_handle);

	if (robot_type == omniplatform) {
		for (int i = 0; i < 4; i++) {
			sprintf(str, "%s/link[%d]/regularRotation", robot_name.c_str(), i);
			simxGetObjectHandle(clientID, str, &motor_handle[i], simx_opmode_blocking);
			printf("   Motor Handle [%d] = %d \n", i, motor_handle[i]);
		}
	}

	else if (robot_type == pioneer) {
		sprintf(str, "%s/leftMotor", robot_name.c_str());
		simxGetObjectHandle(clientID, str, &motor_handle[0], simx_opmode_blocking);
		sprintf(str, "%s/rightMotor", robot_name.c_str());
		simxGetObjectHandle(clientID, str, &motor_handle[1], simx_opmode_blocking);

		printf("   Motor Handle [0] = %d \n", motor_handle[0]);
		printf("   Motor Handle [1] = %d \n", motor_handle[1]);
	}

	return robot_handle;
}

void CoppeliaRobotMobile::move(float v_left, float v_right) {
	
	v_left /= 1000;		// mm/s
	v_right /= 1000;	// mm/s

	if (robot_type == pioneer) {
		simxSetJointTargetVelocity(clientID, motor_handle[0], v_left, simx_opmode_oneshot);
		simxSetJointTargetVelocity(clientID, motor_handle[1], v_right, simx_opmode_oneshot);
	}

}



//////////////////////////////////////////////////////////////////
//						Coppelia Sensor						    //
//////////////////////////////////////////////////////////////////

CoppeliaSensor::CoppeliaSensor(int type, string name) 
{
	sensor_type = type;
	sensor_name = name;
}

int CoppeliaSensor::init() 
{
	int ret = simxGetObjectHandle(clientID, sensor_name.c_str(), &sensor_handle, simx_opmode_blocking);
	if (ret != simx_return_ok) return 0;

	if (sensor_handle != 0) {
		if (sensor_type == vision_sensor) {
			simxGetVisionSensorImage(clientID, sensor_handle, resolution, &image, 0, simx_opmode_streaming);
		}
		else if (sensor_type == proximity_sensor) {
			simxUChar state;
			float objPoint[3];
			float objNorm[3];
			int objHandle;
			simxReadProximitySensor(clientID, sensor_handle,&state, objPoint, &objHandle, objNorm, simx_opmode_streaming);
		}
		else if (sensor_type == force_sensor) {
			simxUChar state;
			float force[3], torque[3];
			simxReadForceSensor(clientID, sensor_handle, &state, force, torque, simx_opmode_streaming);
		}
	}
	else {
		printf("Failed to initialize sensor!!! \n");
	}
	return sensor_handle;
}

void CoppeliaSensor::get_image(simxUChar** img, int res[2]) 
{
	simxGetVisionSensorImage(clientID, sensor_handle, res, img, 0, simx_opmode_buffer);
}

int CoppeliaSensor::get_state() 
{
	int ret = simxReadProximitySensor(clientID, sensor_handle, &data.state, data.objPoint, &data.objHandle, data.objNorm, simx_opmode_buffer);
	return (int) data.state;
}

void CoppeliaSensor::readForce(float dataForce[6]) 
{
	simxReadForceSensor(clientID, sensor_handle, &state, force, torque, simx_opmode_buffer);
	for (int i = 0; i < 3; i++) {
		dataForce[i] = force[i];
		dataForce[i + 3] = torque[i];
	}
}

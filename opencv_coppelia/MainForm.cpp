#include "MainForm.h"
#include "include/coppeliasim.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <process.h>
#include <msclr\marshal_cppstd.h>

using namespace System;
using namespace System::Windows::Forms;
using namespace opencvcoppelia;
using namespace cv;

CoppeliaSim		mSim;
CoppeliaRobot	mRobot ("/UR10");
CoppeliaSensor	camera(vision_sensor, "/eyeinhand");
CoppeliaSensor	proxSensor(proximity_sensor, "/Proximity_sensor");
CoppeliaSensor  forceSensor(force_sensor, "/force_sensor");

int clientId;
int idCam, idProx;

bool imgAvailable = false;
Mat outputImg;


// Convert map tp Hbitmap to update image in the picture box:
Bitmap^ MatToHbitmap(Mat inputImg) {
	Mat src;
	inputImg.copyTo(src);
	cvtColor(src, src, COLOR_RGB2BGRA);
	HBITMAP hBit = CreateBitmap(src.cols, src.rows, 1, 32, src.data);
	Bitmap^ bmp = Bitmap::FromHbitmap((IntPtr)hBit);
	return bmp;
}

// Initialization program
void MainForm::on_init() {
	timer1->Interval = 30;
	timer1->Enabled = true;
}

// Timer 1 tick
void MainForm::on_timer() {
	if (imgAvailable) {
		imgAvailable = false;
		Bitmap^ bmp = MatToHbitmap(outputImg);
		pictureBox1->Image = bmp;
		pictureBox1->SizeMode = PictureBoxSizeMode::Zoom;
	}
}

// Connect To CoppeliaSim and Initialize Objects 
void MainForm::btn_sim_connect() {
	
	if (mSim.connect(19997))
		mSim.startSimulation();
	
	camera.init();
	proxSensor.init();
	mRobot.init();
	forceSensor.init();
}


// Read Proximity Sensor
void read_proximity(void*) {
	while (1) {
		int state = proxSensor.get_state();
		if (state) {
			printf("object detected: %d (%.2f, %.2f, %.2f) \n", proxSensor.data.objHandle, proxSensor.data.objPoint[0], proxSensor.data.objPoint[1], proxSensor.data.objPoint[2]);
		}
		Sleep(100);
	}
}
void MainForm::btn_read_proximity() {
	//_beginthread(read_proximity, 0, NULL);
	int state = proxSensor.get_state();
	if (state) {
		printf("object detected: %d (%.2f, %.2f, %.2f) \n", proxSensor.data.objHandle, proxSensor.data.objPoint[0], proxSensor.data.objPoint[1], proxSensor.data.objPoint[2]);
	}
	else {
		printf("No object detected \n");
	}
}


// Read Vision Sensor
void read_vision_sensor(void*) {
	simxUChar* image = 0;
	int res[2];
	while (1) {
		camera.get_image(&image, res);
		Mat img(res[1], res[0], CV_8UC3, image);
		flip(img, img, 0);
		imshow("image", img);
		img.copyTo(outputImg);
		//if (outputImg.data != nullptr) imgAvailable = true;
		//else imgAvailable = false;
		waitKey(30);
	}
}

void MainForm::btn_sim_read_camera() {
	// _beginthread(read_vision_sensor, 0, NULL);
	int resolution[2];
	simxUChar* image;
	camera.get_image(&image, resolution);
	Mat img(resolution[1], resolution[0], CV_8UC3, image);
	flip(img, img, 0);
	cvtColor(img, img, COLOR_BGR2RGB);
	imshow("image", img);
	img.copyTo(outputImg);
	if (outputImg.data != nullptr) imgAvailable = true;
	else imgAvailable = false;
}

void MainForm::btn_read_robot_joint_pos() {
	float pos[6];
	mRobot.readJointPosition(pos);
	printf("Robot joint pos: %.3f, %.3f, %.3f \n", pos[0], pos[1], pos[2]);
}

void MainForm::btn_read_force_sensor() {
	float force[6];
	forceSensor.readForce(force);
	printf("Robot F/T sensor: %.3f, %.3f, %.3f \n", force[0], force[1], force[2]);
}

void MainForm::btn_remove_object() {
	//int handle = Convert::ToInt16(textBox1->Text);
	//simxRemoveObject(clientId, handle, simx_opmode_oneshot_wait);
}

void MainForm::btn_add_object() {
	int target_handle, pos_handle;
	float pos[3];
	simxGetObjectHandle(clientId, "/Cuboid", &target_handle, simx_opmode_blocking);
	simxGetObjectHandle(clientId, "/object_position", &pos_handle, simx_opmode_blocking);
	simxGetObjectPosition(clientId, pos_handle, -1, pos, simx_opmode_blocking);
	mSim.addObject(target_handle, pos);
}


// Read Robot Position and data
void MainForm::btn_read_robot_pos() {
	float pos[6];
	mRobot.readPosition(pos);
	printf("Robot pos: %.3f, %.3f, %.3f \n", pos[0], pos[1], pos[2]);
}

// Robot set position
void MainForm::btn_set_robot_pos() {
	float pos[6] = { 0, 500, 500, 180, 0, 0 };
	mRobot.setPosition(pos, false);
}
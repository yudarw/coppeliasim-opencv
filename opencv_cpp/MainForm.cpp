#include "MainForm.h"
#include <process.h>
#include <msclr\marshal_cppstd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>


using namespace System;
using namespace System::Windows::Forms;
using namespace opencvcpp;
using namespace cv;

bool imgAvailable = false;
Mat outputImg;

// Convert map tp Hbitmap to update image in the picture box:
Bitmap^ MatToHbitmap(Mat inputImg) {
	Mat src;
	inputImg.copyTo(src);
	cvtColor(src, src, COLOR_BGR2BGRA);
	HBITMAP hBit = CreateBitmap(src.cols, src.rows, 1, 32, src.data);
	Bitmap^ bmp = Bitmap::FromHbitmap((IntPtr)hBit);
	return bmp;
}

// Show image
void MainForm::btn_open_camera() {
	Mat img = imread("images/labmates.jpeg", IMREAD_COLOR);
	//imshow("Image", img);
	Bitmap^ bmp = MatToHbitmap(img);
	pictureBox1->Image = bmp;
	pictureBox1->SizeMode = PictureBoxSizeMode::Zoom;
}


void thread_video_capture(void*) {
	VideoCapture cap(1);
	if (!cap.isOpened()) {
		printf("Cannot open camera \n");
	}
	while (1) {
		cap >> outputImg;
		imshow("image", outputImg);
		if (outputImg.data != nullptr) imgAvailable = true;
		else imgAvailable = false;
		waitKey(25);
	}
}

// Streaming web camera
void MainForm::btn_video_capture() {
	_beginthread(thread_video_capture, 0, NULL);
}



// On initialization
void MainForm::on_init() { 
	timer1->Interval = 100;
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



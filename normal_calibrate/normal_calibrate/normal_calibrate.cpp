/***********Edit by WangZheng***********/
/************Calibration Test***********/
/*************2016--10--30**************/
#include <opencv2\opencv.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include "videoInput.h"

using namespace std;
void help(){
	printf("\n\n"
		" Calling convention:\n"
		" ch11_ex11_1  board_w  board_h  number_of_boards  skip_frames\n"
		"\n"
		"   WHERE:\n"
		"     board_w, board_h   -- are the number of corners along the row and columns respectively\n"
		"     number_of_boards   -- are the number of chessboard views to collect before calibration\n"
		"     skip_frames        -- are the number of frames to skip before trying to collect another\n"
		"                           good chessboard.  This allows you time to move the chessboard.  \n"
		"                           Move it to many different locations and angles so that calibration \n"
		"                           space will be well covered. \n"
		"\n"
		" Hit ‘p’ to pause/unpause, ESC to quit\n"
		"\n");
}

int n_boards = 10; //Will be set by input list
int board_dt = 90; //Wait 90 frames per chessboard view
int board_w = 7;
int board_h = 5;

int main() {
	int deviceNum = 0;
	videoInput VI;
	int deviceNums = VI.listDevices();
	cout << "You have " << deviceNums << " avaliable!" << endl;
	VI.setUseCallback(true);
	VI.setIdealFramerate(deviceNum, 60);
	//VI.setupDevice(deviceNum,640,480,VI_COMPOSITE);
	VI.setupDevice(deviceNum, 640, 480);
	int width = VI.getWidth(deviceNum);
	int height = VI.getHeight(deviceNum);
	cout << "Present paras as " << "width=" << width << "\t" << "height=" << height << endl;
	VI.showSettingsWindow(0);//该语句可以显示视频设置窗口，可以去掉
	IplImage* image = cvCreateImage(cvSize(width, height), 8, 3);//显示的图像
	IplImage* img_frame = cvCreateImage(cvSize(width, height), 8, 3);//缓存的图像
	int framesize = VI.getSize(deviceNum);
	cout << "framesize=" << framesize << endl;
	unsigned char* frameBuffer = (unsigned char*)malloc(framesize);
	char *str1;
	str1 = ".jpg";
	char filename[2][20] = { "", "" };

//	CvCapture* capture;// = cvCreateCameraCapture( 0 );
	// assert( capture );

	int board_n = board_w * board_h;
	CvSize board_sz = cvSize(board_w, board_h);
//	capture = cvCreateCameraCapture(0);
//	if (!capture) { printf("\nCouldn't open the camera\n"); help(); return -1; }

//	cvNamedWindow("Calibration");
//	cvNamedWindow("Raw Video");
	//ALLOCATE STORAGE
	CvMat* image_points = cvCreateMat(n_boards*board_n, 2, CV_32FC1);
	CvMat* object_points = cvCreateMat(n_boards*board_n, 3, CV_32FC1);
	CvMat* point_counts = cvCreateMat(n_boards, 1, CV_32SC1);
	CvMat* intrinsic_matrix = cvCreateMat(3, 3, CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(4, 1, CV_32FC1);

	CvPoint2D32f* corners = new CvPoint2D32f[board_n];
	int corner_count;
	int successes = 0;
	int step, frame = 0;

//	IplImage *image = cvQueryFrame(capture);
	IplImage *gray_image = cvCreateImage(cvGetSize(image), 8, 1);//subpixel

	// CAPTURE CORNER VIEWS LOOP UNTIL WE’VE GOT n_boards 
	// SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND)
	//
	help();
	while (successes < n_boards) {
		VI.getPixels(deviceNum, frameBuffer, false, false);//不能使用videoInput内置的翻转，内存冲突，暂未解决
		img_frame->imageData = (char*)frameBuffer;
		cvFlip(img_frame, image, 0);//0竖直翻转1水平翻转-1垂直水平翻转
//		cvShowImage("原始图像", image);
		//Skip every board_dt frames to allow user to move chessboard
		if ((frame++ % board_dt) == 0) {
			//Find chessboard corners:
			int found = cvFindChessboardCorners(
				image, board_sz, corners, &corner_count,
				CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
				);

			//Get Subpixel accuracy on those corners
			cvCvtColor(image, gray_image, CV_BGR2GRAY);
			cvFindCornerSubPix(gray_image, corners, corner_count,
				cvSize(11, 11), cvSize(-1, -1), cvTermCriteria(
				CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			//Draw it
			cvDrawChessboardCorners(image, board_sz, corners,
				corner_count, found);
			//      cvShowImage( "Calibration", image );

			// If we got a good board, add it to our data
			if (corner_count == board_n) {
				cvShowImage("Calibration", image); //show in color if we did collect the image
//				cvSaveImage("Calibration", image);
				step = successes*board_n;
				for (int i = step, j = 0; j<board_n; ++i, ++j) {
					CV_MAT_ELEM(*image_points, float, i, 0) = corners[j].x;
					CV_MAT_ELEM(*image_points, float, i, 1) = corners[j].y;
					CV_MAT_ELEM(*object_points, float, i, 0) = j / board_w;
					CV_MAT_ELEM(*object_points, float, i, 1) = j%board_w;
					CV_MAT_ELEM(*object_points, float, i, 2) = 0.0f;
				}
				CV_MAT_ELEM(*point_counts, int, successes, 0) = board_n;
				successes++;
				printf("Collected our %d of %d needed chessboard images\n", successes, n_boards);
			}
			else
				cvShowImage("Calibration", gray_image); //Show Gray if we didn't collect the image
		} //end skip board_dt between chessboard capture

		//Handle pause/unpause and ESC
		int c = cvWaitKey(15);
		if (c == 'p'){
			c = 0;
			while (c != 'p' && c != 27){
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
			return 0;
//		image = cvQueryFrame(capture); //Get next image
		cvShowImage("Raw Video", image);
	} //END COLLECTION WHILE LOOP.
	cvDestroyWindow("Calibration");
	printf("\n\n*** CALLIBRATING THE CAMERA...");
	//ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
	CvMat* object_points2 = cvCreateMat(successes*board_n, 3, CV_32FC1);
	CvMat* image_points2 = cvCreateMat(successes*board_n, 2, CV_32FC1);
	CvMat* point_counts2 = cvCreateMat(successes, 1, CV_32SC1);
	//TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
	for (int i = 0; i<successes*board_n; ++i){
		CV_MAT_ELEM(*image_points2, float, i, 0) =
			CV_MAT_ELEM(*image_points, float, i, 0);
		CV_MAT_ELEM(*image_points2, float, i, 1) =
			CV_MAT_ELEM(*image_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 0) =
			CV_MAT_ELEM(*object_points, float, i, 0);
		CV_MAT_ELEM(*object_points2, float, i, 1) =
			CV_MAT_ELEM(*object_points, float, i, 1);
		CV_MAT_ELEM(*object_points2, float, i, 2) =
			CV_MAT_ELEM(*object_points, float, i, 2);
	}
	for (int i = 0; i<successes; ++i){ //These are all the same number
		CV_MAT_ELEM(*point_counts2, int, i, 0) =
			CV_MAT_ELEM(*point_counts, int, i, 0);
	}
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);

	// At this point we have all of the chessboard corners we need.
	// Initialize the intrinsic matrix such that the two focal
	// lengths have a ratio of 1.0
	//
	CV_MAT_ELEM(*intrinsic_matrix, float, 0, 0) = 1.0f;
	CV_MAT_ELEM(*intrinsic_matrix, float, 1, 1) = 1.0f;

	//CALIBRATE THE CAMERA!
	cvCalibrateCamera2(
		object_points2, image_points2,
		point_counts2, cvGetSize(image),
		intrinsic_matrix, distortion_coeffs,
		NULL, NULL, 0  //CV_CALIB_FIX_ASPECT_RATIO
		);

	// SAVE THE INTRINSICS AND DISTORTIONS
	printf(" *** DONE!\n\nStoring Intrinsics.xml and Distortions.xml files\n\n");
	cvSave("Intrinsics.xml", intrinsic_matrix);
	cvSave("Distortion.xml", distortion_coeffs);

	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
	CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");

	// Build the undistort map which we will use for all 
	// subsequent frames.
	//
	IplImage* mapx = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	IplImage* mapy = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 1);
	cvInitUndistortMap(
		intrinsic,
		distortion,
		mapx,
		mapy
		);
	// Just run the camera to the screen, now showing the raw and
	// the undistorted image.
	//
//	cvNamedWindow("Undistort");
	while (image) {
		IplImage *t = cvCloneImage(image);
		cvShowImage("Raw Video", image); // Show raw image
		cvRemap(t, image, mapx, mapy);     // Undistort image
		cvReleaseImage(&t);
		cvShowImage("Undistort", image);     // Show corrected image

		//Handle pause/unpause and ESC
		int c = cvWaitKey(15);
		if (c == 'p'){
			c = 0;
			while (c != 'p' && c != 27){
				c = cvWaitKey(250);
			}
		}
		if (c == 27)
			break;
//		image = cvQueryFrame(capture);
		VI.getPixels(deviceNum, frameBuffer, false, false);//不能使用videoInput内置的翻转，内存冲突，暂未解决
		img_frame->imageData = (char*)frameBuffer;
		cvFlip(img_frame, image, 0);//0竖直翻转1水平翻转-1垂直水平翻转
	}
	VI.stopDevice(deviceNum);
	cvReleaseImage(&image);
	cvReleaseImage(&img_frame);
	return 0;
}




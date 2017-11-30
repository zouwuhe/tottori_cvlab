//Opencv 1.0 is used in this program.
//The purpose of this project is to calibrate the internal parameters of  RGB camera and IR camera of kinect1 with some different positions of one chess board.
//Then save the intrinsic_matrix and distortion_coeffs of kinect1's RGB camera and IR camera.
//First calibrate the internal parameters of RGB camera then calibrate the internal parameters of IR camera.
//After kinect calibrated, first show the calibration result of RGB camera then show the result of IR camera.
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <cxcore.h>
#include <NuiApi.h>
#include <iostream>

using namespace std;

int n_boards=0;//will be set by input list
const int board_dt=30;//wait 20 frames per chessboard view
int board_w;//the width of chessboard
int board_h;//the highth of chessboard
float angle;//the anlge of kinect1

int main(int argc,char* argv[]){
	cout<<"Input these parameters:"<<endl;
	cout<<"1. the width of chessboard"<<endl;
	cout<<"2. the highth of chessboard"<<endl;
	cout<<"3. the total number of images used for calibration"<<endl;
	cout<<"4. the angle of kinect1"<<endl;
	cin>>board_w>>board_h>>n_boards>>angle;
	//read in the data from the RGB camera of kinect1 //先启动 kinect1，并打开kinect1数据流，然后读取RGB图像
	cvNamedWindow("RGB frame");
	cvNamedWindow("calibrate RGB");
	IplImage *colorframe=cvCreateImage(cvSize(1280,960),8,3); // (640,480)
	///initialize
	HRESULT hr=NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_DEPTH|NUI_INITIALIZE_FLAG_USES_SKELETON);///NUI_INITIALIZE_FLAG_USES_DEPTH can be changed to NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
	if(hr!=S_OK){
		cout<<"NuiInitialize failed\n"<<endl;
		return hr;
	}
	//adjust the angle of the kinect1
	NuiCameraElevationSetAngle(angle);
	///open the color frame stream
	HANDLE hNextColorFrameEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	HANDLE pColorStreamHandle=NULL;
	hr=NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_1280x960,//NUI_IMAGE_RESOLUTION_640x480
		0,
		2,
		hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
		&pColorStreamHandle);//打开的彩色数据流的句柄的地址
	if(FAILED(hr)){
		cout<<"Can't open the color frame stream\n"<<endl;
		return hr;
	}
	//prepare for the collection of chessboard
	int board_n=board_w*board_h;
	CvSize board_sz=cvSize(board_w,board_h);
	int corner_count;
	int step,frame=0;
	IplImage *image=cvCreateImage(cvGetSize(colorframe),8,3);
	IplImage *gray_image=cvCreateImage(cvGetSize(colorframe),8,1);
	///////////////////////////////////////////////////////calibrate RGB camera///////////////////////////////////////////////////////////////////////////
	int RGB_successes=0;
	//allocate storage for calibrating RGB camera
	CvMat *image_points=cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat *object_points=cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat *point_counts=cvCreateMat(n_boards,1,CV_32FC1);
	CvMat *intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *distortion_coeffs=cvCreateMat(4,1,CV_32FC1);
	CvPoint2D32f *corners=new CvPoint2D32f[board_n];
	/////////////main loop for calibrating RGB camera
	while(1){
		//read in the rgb image in real time.
		if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
			///read the color frame from color data stream
			const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for color data
			hr=NuiImageStreamGetNextFrame(
				pColorStreamHandle,
				0,//不需继续等待新图像帧而直接返回的时间
				&pImageFrame);//从指定数据流获取下一帧信息，信息地址存放在NUI_IMAGE_FRAME的结构体指针，结构体中的INuiFrameTexture包含帧数据
			if(FAILED(hr)){
				cout<<"Get color frame failed\n"<<endl;
				continue;
			}
			INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //从NUI_IMAGE_FRAME结构体中，提取出需要的帧数据
			NUI_LOCKED_RECT lockedrect;//定义一个结构体，用于保存锁定区域的信息
			pTexture->LockRect(0,&lockedrect,NULL,0);
			if(lockedrect.Pitch!=0){//pitch返回缓冲区中一行的字节数
				BYTE *pBuffer=(BYTE*)lockedrect.pBits;//缓冲区首地址
				for(int y=0;y<colorframe->height;y++){
					uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
						colorframe->imageData+y*colorframe->widthStep
						);
					for(int x=0;x<colorframe->width;x++){
						for(int z=0;z<3;z++){
							color_ptr[3*x+z]=pBuffer[y*4*colorframe->width+x*4+z];
						}
					}
				}
				cvShowImage("RGB frame",colorframe);
				cvWaitKey(33);
			}
			else{
				cout<<"Bufferlength of received texture from color buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(pColorStreamHandle,pImageFrame);
		}
		cvCopyImage(colorframe,image);
		//collection of chessboard
		if(frame++%board_dt==0){//每间隔20帧尝试一次 采集图像上的棋盘角点
			// find chessboard corners
			int found=cvFindChessboardCorners(
				image,board_sz,corners,&corner_count,
				CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS
				);
			//Get Subpixel accuracy on those corners
			cvCvtColor(image,gray_image,CV_RGB2GRAY);
			cvFindCornerSubPix(gray_image,corners,corner_count,
				cvSize(11,11),cvSize(-1,-1),cvTermCriteria(
				CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			//draw it
			cvDrawChessboardCorners(image,board_sz,corners,
				corner_count,found);
			cvShowImage("calibrate RGB",image);
			cvWaitKey(33);//显示图像是需要时间的，所以必须给一个延时，否则无法同步看到图像
			//if we got a good board, add it to our data
			if(corner_count==board_n){
				step=RGB_successes*board_n;
				for(int i=step,j=0;j<board_n;++i,++j){
					CV_MAT_ELEM(*image_points,float,i,0)=corners[j].x;
					CV_MAT_ELEM(*image_points,float,i,1)=image->height-corners[j].y;
					CV_MAT_ELEM(*object_points,float,i,0)=j/board_w;
					CV_MAT_ELEM(*object_points,float,i,1)=j%board_w;
					CV_MAT_ELEM(*object_points,float,i,2)=0.0f;
				}
				CV_MAT_ELEM(*point_counts,int,RGB_successes,0)=board_n;
				RGB_successes++;
			}
		}
		if(RGB_successes>=n_boards)
			break;
		//handle pause/unpause and ESC
		int c=cvWaitKey(15);
		if(c=='p'){
			c=0;
			while(c!='p'&&c!=27){
				c=cvWaitKey(250);
			}
		}
		if(c==27){
			NuiShutdown();
			return 0;
		}
	}
	//allocate matrices according to how many chessboards found
	CvMat *object_points2=cvCreateMat(RGB_successes*board_n,3,CV_32FC1);
	CvMat *image_points2=cvCreateMat(RGB_successes*board_n,2,CV_32FC1);
	CvMat *point_counts2=cvCreateMat(RGB_successes,1,CV_32SC1);
	//transfer the points into the correct size matrices
	for(int i=0;i<RGB_successes*board_n;++i){
		CV_MAT_ELEM(*image_points2,float,i,0)=CV_MAT_ELEM(*image_points,float,i,0);
		CV_MAT_ELEM(*image_points2,float,i,1)=CV_MAT_ELEM(*image_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,0)=CV_MAT_ELEM(*object_points,float,i,0);
		CV_MAT_ELEM(*object_points2,float,i,1)=CV_MAT_ELEM(*object_points,float,i,1);
		CV_MAT_ELEM(*object_points2,float,i,2)=CV_MAT_ELEM(*object_points,float,i,2);
	}
	for(int i=0;i<RGB_successes;++i){
		CV_MAT_ELEM(*point_counts2,int,i,0)=CV_MAT_ELEM(*point_counts,int,i,0);
	}
	delete [] corners;
	cvReleaseMat(&object_points);
	cvReleaseMat(&image_points);
	cvReleaseMat(&point_counts);
	//at this point, we have all of the chessboard corners we need.
	//initialize the intrinsic matrix such that the two focal lengths have a ratio of 1.0
	CV_MAT_ELEM(*intrinsic_matrix,float,0,0)=1.0f;
	CV_MAT_ELEM(*intrinsic_matrix,float,1,1)=1.0f;
	//calibrate the rgb camera of kinect1 
	cvCalibrateCamera2(
		object_points2,
		image_points2,
		point_counts2,
		cvGetSize(image),
		intrinsic_matrix,
		distortion_coeffs,
		NULL,
		NULL,
		0
		);
	//save the intrinsics and distortions
	cvSave("Intrinsic_RGB2.xml",intrinsic_matrix);
	cvSave("Distortion_RGB2.xml",distortion_coeffs);
	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&point_counts2);
	cvReleaseMat(&intrinsic_matrix);
	cvReleaseMat(&distortion_coeffs);
	cvDestroyWindow("RGB frame");
	cvDestroyWindow("calibrate RGB");
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////calibrate IR camera///////////////////////////////////////////////////////////////////////////
	int IR_successes=0;
	cvNamedWindow("IR frame");
	cvNamedWindow("calibrate IR");

	IplImage *IRframe=cvCreateImage(cvSize(640,480),8,3); // (320,240)

	IplImage *IRimage=cvCreateImage(cvGetSize(IRframe),8,3);
	IplImage *IRgray_image=cvCreateImage(cvGetSize(IRframe),8,1);

	//open the infrared frame stream
	hr=NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR_INFRARED,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
		&pColorStreamHandle);//打开的彩色数据流的句柄的地址
	if(FAILED(hr)){
		cout<<"Can't open the infrared frame stream\n"<<endl;
		return hr;
	}
	//allocate storage for calibrating IR camera
	CvMat *IR_image_points=cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat *IR_object_points=cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat *IR_point_counts=cvCreateMat(n_boards,1,CV_32FC1);
	CvMat *IR_intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *IR_distortion_coeffs=cvCreateMat(4,1,CV_32FC1);
	CvPoint2D32f *IR_corners=new CvPoint2D32f[board_n];
	Sleep(3000);
	/////////////main loop for calibrating IR camera
	while(1){
		//read in the rgb image in real time.
		if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
			///read the color frame from color data stream
			const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for color data
			hr=NuiImageStreamGetNextFrame(
				pColorStreamHandle,
				0,//不需继续等待新图像帧而直接返回的时间
				&pImageFrame);//从指定数据流获取下一帧信息，信息地址存放在NUI_IMAGE_FRAME的结构体指针，结构体中的INuiFrameTexture包含帧数据
			if(FAILED(hr)){
				cout<<"Get infrared frame failed\n"<<endl;
				continue;
			}
			INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //从NUI_IMAGE_FRAME结构体中，提取出需要的帧数据
			NUI_LOCKED_RECT lockedrect;//定义一个结构体，用于保存锁定区域的信息
			pTexture->LockRect(0,&lockedrect,NULL,0);
			if(lockedrect.Pitch!=0){//pitch返回缓冲区中一行的字节数
				BYTE *pBuffer=(BYTE*)lockedrect.pBits;//缓冲区首地址
				for(int y=0;y<IRframe->height;y++){
					uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
						IRframe->imageData+y*IRframe->widthStep
						);
					for(int x=0;x<IRframe->width;x++){
						BYTE intensity = reinterpret_cast<USHORT*>(lockedrect.pBits)[y*IRframe->width+x] >> 8;//针对infrared数据流的读取方式（区别与彩色数据流的读取方式）
						for(int z=0;z<3;z++){
							color_ptr[3*x+z]=intensity;
						}
					}
				}
				cvShowImage("IR frame",IRframe);
				cvWaitKey(33);
			}
			else{
				cout<<"Bufferlength of received texture from infrared buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(pColorStreamHandle,pImageFrame);
		}
		cvCopyImage(IRframe,IRimage);
		//collection of chessboard
		if(frame++%board_dt==0){//每间隔20帧尝试一次 采集图像上的棋盘角点
			// find chessboard corners
			int found=cvFindChessboardCorners(
				IRimage,board_sz,IR_corners,&corner_count,
				CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS
				);
			//Get Subpixel accuracy on those corners
			cvCvtColor(IRimage,IRgray_image,CV_RGB2GRAY);
			cvFindCornerSubPix(IRgray_image,IR_corners,corner_count,
				cvSize(11,11),cvSize(-1,-1),cvTermCriteria(
				CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
			//draw it
			cvDrawChessboardCorners(IRimage,board_sz,IR_corners,
				corner_count,found);
			cvShowImage("calibrate IR",IRimage);
			cvWaitKey(33);//显示图像是需要时间的，所以必须给一个延时，否则无法同步看到图像
			//if we got a good board, add it to our data
			if(corner_count==board_n){
				step=IR_successes*board_n;
				for(int i=step,j=0;j<board_n;++i,++j){
					CV_MAT_ELEM(*IR_image_points,float,i,0)=IR_corners[j].x;
					CV_MAT_ELEM(*IR_image_points,float,i,1)=IRimage->height-IR_corners[j].y;
					CV_MAT_ELEM(*IR_object_points,float,i,0)=j/board_w;
					CV_MAT_ELEM(*IR_object_points,float,i,1)=j%board_w;
					CV_MAT_ELEM(*IR_object_points,float,i,2)=0.0f;
				}
				CV_MAT_ELEM(*IR_point_counts,int,IR_successes,0)=board_n;
				IR_successes++;
			}
		}
		if(IR_successes>=n_boards)
			break;
		//handle pause/unpause and ESC
		int c=cvWaitKey(15);
		if(c=='p'){
			c=0;
			while(c!='p'&&c!=27){
				c=cvWaitKey(250);
			}
		}
		if(c==27){
			NuiShutdown();
			return 0;
		}
	}
	//allocate matrices according to how many chessboards found
	CvMat *IR_object_points2=cvCreateMat(IR_successes*board_n,3,CV_32FC1);
	CvMat *IR_image_points2=cvCreateMat(IR_successes*board_n,2,CV_32FC1);
	CvMat *IR_point_counts2=cvCreateMat(IR_successes,1,CV_32SC1);
	//transfer the points into the correct size matrices
	for(int i=0;i<IR_successes*board_n;++i){
		CV_MAT_ELEM(*IR_image_points2,float,i,0)=CV_MAT_ELEM(*IR_image_points,float,i,0);
		CV_MAT_ELEM(*IR_image_points2,float,i,1)=CV_MAT_ELEM(*IR_image_points,float,i,1);
		CV_MAT_ELEM(*IR_object_points2,float,i,0)=CV_MAT_ELEM(*IR_object_points,float,i,0);
		CV_MAT_ELEM(*IR_object_points2,float,i,1)=CV_MAT_ELEM(*IR_object_points,float,i,1);
		CV_MAT_ELEM(*IR_object_points2,float,i,2)=CV_MAT_ELEM(*IR_object_points,float,i,2);
	}
	for(int i=0;i<IR_successes;++i){
		CV_MAT_ELEM(*IR_point_counts2,int,i,0)=CV_MAT_ELEM(*IR_point_counts,int,i,0);
	}
	delete [] IR_corners;
	cvReleaseMat(&IR_object_points);
	cvReleaseMat(&IR_image_points);
	cvReleaseMat(&IR_point_counts);
	//at this point, we have all of the chessboard corners we need.
	//initialize the intrinsic matrix such that the two focal lengths have a ratio of 1.0
	CV_MAT_ELEM(*IR_intrinsic_matrix,float,0,0)=1.0f;
	CV_MAT_ELEM(*IR_intrinsic_matrix,float,1,1)=1.0f;
	//calibrate the rgb camera of kinect1 
	cvCalibrateCamera2(
		IR_object_points2,
		IR_image_points2,
		IR_point_counts2,
		cvGetSize(IRimage),
		IR_intrinsic_matrix,
		IR_distortion_coeffs,
		NULL,
		NULL,
		0
		);
	//save the intrinsics and distortions
	cvSave("Intrinsic_IR2.xml",IR_intrinsic_matrix);
	cvSave("Distortion_IR2.xml",IR_distortion_coeffs);
	cvReleaseMat(&IR_object_points2);
	cvReleaseMat(&IR_image_points2);
	cvReleaseMat(&IR_point_counts2);
	cvReleaseMat(&IR_intrinsic_matrix);
	cvReleaseMat(&IR_distortion_coeffs);
	cvDestroyWindow("IR frame");
	cvDestroyWindow("calibrate IR");

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	///////////////////////////////////////////////////////show calibration result of RGB camera//////////////////////////////////////////////////////////
	///open the color frame stream
	hr=NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_1280x960,//NUI_IMAGE_RESOLUTION_640x480
		0,
		2,
		hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
		&pColorStreamHandle);//打开的彩色数据流的句柄的地址
	if(FAILED(hr)){
		cout<<"Can't open the color frame stream\n"<<endl;
		return hr;
	}
	//load these matrices back in
	CvMat *RGB_intrinsic=(CvMat*)cvLoad("Intrinsic_RGB2.xml");
	CvMat *RGB_distortion=(CvMat*)cvLoad("Distortion_RGB2.xml");
	//build the undistort map 
	IplImage *RGB_mapx=cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	IplImage *RGB_mapy=cvCreateImage(cvGetSize(image),IPL_DEPTH_32F,1);
	cvInitUndistortMap(
		RGB_intrinsic,
		RGB_distortion,
		RGB_mapx,
		RGB_mapy
		);
	//show the raw and the undistorted image
	cvReleaseMat(&RGB_intrinsic);
	cvReleaseMat(&RGB_distortion);
	cvNamedWindow("RGB_undistort");
	cvNamedWindow("RGB_raw");
	while(1){
		//read in the rgb image in real time.
		if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
			///read the color frame from color data stream
			const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for color data
			hr=NuiImageStreamGetNextFrame(
				pColorStreamHandle,
				0,//不需继续等待新图像帧而直接返回的时间
				&pImageFrame);//从指定数据流获取下一帧信息，信息地址存放在NUI_IMAGE_FRAME的结构体指针，结构体中的INuiFrameTexture包含帧数据
			if(FAILED(hr)){
				cout<<"Get color frame failed\n"<<endl;
				continue;
			}
			INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //从NUI_IMAGE_FRAME结构体中，提取出需要的帧数据
			NUI_LOCKED_RECT lockedrect;//定义一个结构体，用于保存锁定区域的信息
			pTexture->LockRect(0,&lockedrect,NULL,0);
			if(lockedrect.Pitch!=0){//pitch返回缓冲区中一行的字节数
				BYTE *pBuffer=(BYTE*)lockedrect.pBits;//缓冲区首地址
				for(int y=0;y<colorframe->height;y++){
					uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
						colorframe->imageData+y*colorframe->widthStep
						);
					for(int x=0;x<colorframe->width;x++){
						for(int z=0;z<3;z++){
							color_ptr[3*x+z]=pBuffer[y*4*colorframe->width+x*4+z];
						}
					}
				}
				//cvShowImage("colorframe",colorframe);
				//cvWaitKey(33);
			}
			else{
				cout<<"Bufferlength of received texture from color buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(pColorStreamHandle,pImageFrame);
		}
		cvShowImage("RGB_raw",colorframe);
		cvRemap(colorframe,image,RGB_mapx,RGB_mapy);
		cvShowImage("RGB_undistort",image);
		//handle pause/unpause and ESC
		int c=cvWaitKey(15);
		if(c=='p'){
			c=0;
			while(c!='p'&&c!=27){
				c=cvWaitKey(250);
			}
		}
		if(c==27){
			//cvSaveImage("kinect1_RGB_raw.jpg",colorframe);
			//cvSaveImage("kinect1_RGB_undistort.jpg",image);
			//cvReleaseImage(&RGB_mapx);
			//cvReleaseImage(&RGB_mapy);
			//cvReleaseImage(&colorframe);
			//cvReleaseImage(&image);
			//NuiShutdown();
			//return 0;
			break;
		}
	}
	cvSaveImage("kinect2_RGB_raw.jpg",colorframe);
	cvSaveImage("kinect2_RGB_undistort.jpg",image);
	cvReleaseImage(&RGB_mapx);
	cvReleaseImage(&RGB_mapy);
	cvDestroyWindow("RGB_undistort");
	cvDestroyWindow("RGB_raw");
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////show calibration result of IR camera//////////////////////////////////////////////////////////
	///open the infrared frame stream
	hr=NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR_INFRARED,
		NUI_IMAGE_RESOLUTION_640x480, //NUI_IMAGE_RESOLUTION_320x240
		0,
		2,
		hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
		&pColorStreamHandle);//打开的彩色数据流的句柄的地址
	if(FAILED(hr)){
		cout<<"Can't open the color frame stream\n"<<endl;
		return hr;
	}
	//load these matrices back in
	CvMat *IR_intrinsic=(CvMat*)cvLoad("Intrinsic_IR2.xml");
	CvMat *IR_distortion=(CvMat*)cvLoad("Distortion_IR2.xml");
	//build the undistort map 
	IplImage *IR_mapx=cvCreateImage(cvGetSize(IRframe),IPL_DEPTH_32F,1);
	IplImage *IR_mapy=cvCreateImage(cvGetSize(IRframe),IPL_DEPTH_32F,1);
	cvInitUndistortMap(
		IR_intrinsic,
		IR_distortion,
		IR_mapx,
		IR_mapy
		);
	//show the raw and the undistorted image
	cvReleaseMat(&IR_intrinsic);
	cvReleaseMat(&IR_distortion);
	cvNamedWindow("IR_undistort");
	cvNamedWindow("IR_raw");
	while(1){
		//read in the IR image in real time.
		if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
			///read the IR frame from color data stream
			const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for IR data
			hr=NuiImageStreamGetNextFrame(
				pColorStreamHandle,
				0,//不需继续等待新图像帧而直接返回的时间
				&pImageFrame);//从指定数据流获取下一帧信息，信息地址存放在NUI_IMAGE_FRAME的结构体指针，结构体中的INuiFrameTexture包含帧数据
			if(FAILED(hr)){
				cout<<"Get color frame failed\n"<<endl;
				continue;
			}
			INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //从NUI_IMAGE_FRAME结构体中，提取出需要的帧数据
			NUI_LOCKED_RECT lockedrect;//定义一个结构体，用于保存锁定区域的信息
			pTexture->LockRect(0,&lockedrect,NULL,0);
			if(lockedrect.Pitch!=0){//pitch返回缓冲区中一行的字节数
				BYTE *pBuffer=(BYTE*)lockedrect.pBits;//缓冲区首地址
				for(int y=0;y<IRframe->height;y++){
					uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
						IRframe->imageData+y*IRframe->widthStep
						);
					for(int x=0;x<IRframe->width;x++){
						BYTE intensity = reinterpret_cast<USHORT*>(lockedrect.pBits)[y*IRframe->width+x] >> 8;//针对infrared数据流的读取方式（区别与彩色数据流的读取方式）
						for(int z=0;z<3;z++){
							color_ptr[3*x+z]=intensity;
						}
					}
				}
			}
			else{
				cout<<"Bufferlength of received texture from color buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(pColorStreamHandle,pImageFrame);
		}
		cvShowImage("IR_raw",IRframe);
		cvRemap(IRframe,IRimage,IR_mapx,IR_mapy);
		cvShowImage("IR_undistort",IRimage);
		//handle pause/unpause and ESC
		int c=cvWaitKey(15);
		if(c=='p'){
			c=0;
			while(c!='p'&&c!=27){
				c=cvWaitKey(250);
			}
		}
		if(c==27){
			//cvSaveImage("kinect1_IR_raw.jpg",colorframe);
			//cvSaveImage("kinect1_IR_undistort.jpg",image);
			//cvReleaseImage(&IR_mapx);
			//cvReleaseImage(&IR_mapy);
			//cvReleaseImage(&colorframe);
			//cvReleaseImage(&image);
			//NuiShutdown();
			break;
		}
	}
	cvSaveImage("kinect2_IR_raw.jpg",IRframe);
	cvSaveImage("kinect2_IR_undistort.jpg",IRimage);
	cvReleaseImage(&IR_mapx);
	cvReleaseImage(&IR_mapy);
	cvDestroyWindow("IR_undistort");
	cvDestroyWindow("IR_raw");
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	cvReleaseImage(&gray_image);
	cvReleaseImage(&colorframe);
	cvReleaseImage(&image);

	cvReleaseImage(&IRgray_image);
	cvReleaseImage(&IRframe);
	cvReleaseImage(&IRimage);
	///shut down the nui
	NuiShutdown();
	return 0;
}
//注意 克隆函数 cvCloneImage() 每次使用会重新分配空间，因此该函数在循环中要慎重使用
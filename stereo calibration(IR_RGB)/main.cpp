//Opencv 2.3.1 is used in this program.
//The purpose of this project is to calibrate the external parameters of  RGB camera and IR camera of kinect1 with some different positions of one chess board.
//Read in the intrinsic_matrix and distortion_coeffs of kinect1's RGB camera and IR camera, then use these internal parameters to calibrate the external parameters.


//#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"


#include <cv.h>
#include <cxmisc.h>
#include <highgui.h>
#include <cvaux.h>
#include <stdio.h>
#include <stdlib.h>
#include <algorithm>
#include <iterator>
#include <ctype.h>

#include <iostream>
#include <NuiApi.h>
#include <sstream>
#include <string>

using namespace std;
using namespace cv;

int n_boards=0;//will be set by input list
const int board_dt=10;//wait 20 frames per chessboard view
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
	cvNamedWindow("RGB frame");
	cvNamedWindow("IR frame");
	cvNamedWindow("calibrate RGB");
	cvNamedWindow("calibrate IR");
	IplImage *colorframe=cvCreateImage(cvSize(640,480),8,3);
	IplImage *image=cvCreateImage(cvSize(640,480),8,3);
	IplImage *gray_image=cvCreateImage(cvSize(640,480),8,1);
	//prepare for the collection of chessboard
	int board_n=board_w*board_h;
	CvSize board_sz=cvSize(board_w,board_h);
	int corner_count;
	int step,frame=0;
	string filename="Kinect2"; 
	///////////////////////////////////////////////////////////////////字符串操作
	//int RGB_successes=1;
	//stringstream ss;
	//string s1;
	//ss<<RGB_successes;
	//ss>>s1;
	//filename=filename+"_RGB_"+s1+".jpg";
	//const char *file=filename.c_str();
	//cvSaveImage(file,colorframe);

	///////////////////////////////////////////////////////////////////////////
	///initialize
	HRESULT hr=NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_DEPTH|NUI_INITIALIZE_FLAG_USES_SKELETON);///NUI_INITIALIZE_FLAG_USES_DEPTH can be changed to NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
	if(hr!=S_OK){
		cout<<"NuiInitialize failed\n"<<endl;
		return hr;
	}
	HANDLE hNextColorFrameEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	HANDLE pColorStreamHandle=NULL;
	//adjust the angle of the kinect1
	NuiCameraElevationSetAngle((long)angle);
	/////////////////////////////////////////////////////////////////////////

	//allocate storage for calibrating RGB camera
	int RGB_successes=0;
	CvMat *RGB_image_points=cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat *RGB_object_points=cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat *RGB_point_counts=cvCreateMat(n_boards,1,CV_32FC1);
	CvMat *RGB_intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *RGB_distortion_coeffs0=cvCreateMat(4,1,CV_32FC1);//读入的RGB内参标定结果 对应opencv1.0  
	CvMat *RGB_distortion_coeffs=cvCreateMat(1,5,CV_32FC1);//在本程序中用于stereo calibration的畸变参数 对应opencv2.3.1
	CvPoint2D32f *RGB_corners=new CvPoint2D32f[board_n];
	//allocate storage for calibrating IR camera
	int IR_successes=0;
	CvMat *IR_image_points=cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat *IR_object_points=cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat *IR_point_counts=cvCreateMat(n_boards,1,CV_32FC1);
	CvMat *IR_intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *IR_distortion_coeffs0=cvCreateMat(4,1,CV_32FC1);//读入的IR内参标定结果 对应opencv1.0  
	CvMat *IR_distortion_coeffs=cvCreateMat(1,5,CV_32FC1);//在本程序中用于stereo calibration的畸变参数 对应opencv2.3.1
	CvPoint2D32f *IR_corners=new CvPoint2D32f[board_n];

	/////////////////////////////////////////////////////////////Collect data for stereo calibration (IR camera and RGB camera)//////////////////////////////////////////////////////////////////////////
	while(RGB_successes<n_boards||IR_successes<n_boards){
		//////////////////////////////collect image and data from IR camera/////////////////////////////////////////////
		while(1){
			///open the IR frame stream
			hr=NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR_INFRARED,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
				&pColorStreamHandle);//打开的彩色数据流的句柄的地址
			if(FAILED(hr)){
				cout<<"Can't open the IR frame stream\n"<<endl;
				return hr;
			}
			//read in the IR image in real time.
			if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
				///read the IR frame from color data stream
				const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for color data
				hr=NuiImageStreamGetNextFrame(
					pColorStreamHandle,
					0,//不需继续等待新图像帧而直接返回的时间
					&pImageFrame);//从指定数据流获取下一帧信息，信息地址存放在NUI_IMAGE_FRAME的结构体指针，结构体中的INuiFrameTexture包含帧数据
				if(FAILED(hr)){
					cout<<"Get IR frame failed\n"<<endl;
					continue;
				}
				INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //从NUI_IMAGE_FRAME结构体中，提取出需要的帧数据
				NUI_LOCKED_RECT lockedrect;//定义一个结构体，用于保存锁定区域的信息
				pTexture->LockRect(0,&lockedrect,NULL,0);
				if(lockedrect.Pitch!=0){//pitch返回缓冲区中一行的字节数
					BYTE *pBuffer=(BYTE*)lockedrect.pBits;//缓冲区首地址
					for(int y=0;y<480;y++){
						uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
							colorframe->imageData+y*colorframe->widthStep
							);
						for(int x=0;x<640;x++){
							BYTE intensity = reinterpret_cast<USHORT*>(lockedrect.pBits)[y*640+x] >> 8;//针对infrared数据流的读取方式（区别与彩色数据流的读取方式）
							for(int z=0;z<3;z++){
								color_ptr[3*x+z]=intensity;
							}
						}
					}
					cvShowImage("IR frame",colorframe);
					cvWaitKey(33);
				}
				else{
					cout<<"Bufferlength of received texture from IR buffer is bogus\r\n"<<endl;
				}
				///release the data of this frame and be ready for next frame in the color stream
				NuiImageStreamReleaseFrame(pColorStreamHandle,pImageFrame);
			}
			cvCopyImage(colorframe,image);
			//Check whether the pattern is recognized. If it's ok, collect the image of  chessboard.(IR)
			if(frame++%board_dt==0){//每间隔20帧尝试一次 采集图像上的棋盘角点
				// find chessboard corners
				int found=cvFindChessboardCorners(
					image,board_sz,IR_corners,&corner_count,
					CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS
					);
				//Get Subpixel accuracy on those corners
				cvCvtColor(image,gray_image,CV_RGB2GRAY);
				cvFindCornerSubPix(gray_image,IR_corners,corner_count,
					cvSize(11,11),cvSize(-1,-1),cvTermCriteria(
					CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
				//draw it
				cvDrawChessboardCorners(image,board_sz,IR_corners,
					corner_count,found);
				cvShowImage("calibrate IR",image);
				cvWaitKey(33);//显示图像是需要时间的，所以必须给一个延时，否则无法同步看到图像
				//if we got a good board, add it to our data(IR)
				if(corner_count==board_n){
					step=IR_successes*board_n;
					for(int i=step,j=0;j<board_n;++i,++j){
						CV_MAT_ELEM(*IR_image_points,float,i,0)=IR_corners[j].x;
						CV_MAT_ELEM(*IR_image_points,float,i,1)=image->height-IR_corners[j].y;
						CV_MAT_ELEM(*IR_object_points,float,i,0)=j/board_w;
						CV_MAT_ELEM(*IR_object_points,float,i,1)=j%board_w;
						CV_MAT_ELEM(*IR_object_points,float,i,2)=0.0f;
					}
					CV_MAT_ELEM(*IR_point_counts,int,IR_successes,0)=board_n;
					IR_successes++;
					////////////////////////save the useful image
					stringstream ss1;
					string s1;
					ss1<<IR_successes;
					ss1>>s1;
					string file_IR=filename+"_IR_"+s1+".jpg";
					const char *file1=file_IR.c_str();
					cvSaveImage(file1,image);
					///////////////////////
					break;
				}
			}
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
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

		//////////////////////////////collect image and data from RGB camera/////////////////////////////////////////////
		while(1){
			///open the color frame stream
			hr=NuiImageStreamOpen(
				NUI_IMAGE_TYPE_COLOR,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
				&pColorStreamHandle);//打开的彩色数据流的句柄的地址
			if(FAILED(hr)){
				cout<<"Can't open the color frame stream\n"<<endl;
				return hr;
			}
			//read in the RGB image in real time.
			if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
				///read the RGB frame from color data stream
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
					for(int y=0;y<480;y++){
						uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
							colorframe->imageData+y*colorframe->widthStep
							);
						for(int x=0;x<640;x++){
							for(int z=0;z<3;z++){
								color_ptr[3*x+z]=pBuffer[y*4*640+x*4+z];
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
			//Check whether the pattern is recognized. If it's ok, collect the image of  chessboard.(RGB)
			if(frame++%board_dt==0){//每间隔20帧尝试一次 采集图像上的棋盘角点
				// find chessboard corners
				int found=cvFindChessboardCorners(
					image,board_sz,RGB_corners,&corner_count,
					CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS
					);
				//Get Subpixel accuracy on those corners
				cvCvtColor(image,gray_image,CV_RGB2GRAY);
				cvFindCornerSubPix(gray_image,RGB_corners,corner_count,
					cvSize(11,11),cvSize(-1,-1),cvTermCriteria(
					CV_TERMCRIT_EPS+CV_TERMCRIT_ITER,30,0.1));
				//draw it
				cvDrawChessboardCorners(image,board_sz,RGB_corners,
					corner_count,found);
				cvShowImage("calibrate RGB",image);
				cvWaitKey(33);//显示图像是需要时间的，所以必须给一个延时，否则无法同步看到图像
				//if we got a good board, add it to our data
				if(corner_count==board_n){
					step=RGB_successes*board_n;
					for(int i=step,j=0;j<board_n;++i,++j){
						CV_MAT_ELEM(*RGB_image_points,float,i,0)=RGB_corners[j].x;
						CV_MAT_ELEM(*RGB_image_points,float,i,1)=image->height-RGB_corners[j].y;
						CV_MAT_ELEM(*RGB_object_points,float,i,0)=j/board_w;
						CV_MAT_ELEM(*RGB_object_points,float,i,1)=j%board_w;
						CV_MAT_ELEM(*RGB_object_points,float,i,2)=0.0f;
					}
					CV_MAT_ELEM(*RGB_point_counts,int,RGB_successes,0)=board_n;
					RGB_successes++;
					////////////////////////save the useful RGB image
					stringstream ss2;
					string s2;
					ss2<<RGB_successes;
					ss2>>s2;
					string file_RGB=filename+"_RGB_"+s2+".jpg";
					const char *file2=file_RGB.c_str();
					cvSaveImage(file2,image);
					///////////////////////
					break;
				}
			}
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
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		while(1){
			if(cvWaitKey(33)=='c')
				break;
		}
	}

	/*if(RGB_successes!=IR_successes){
		cout<<"...................Data collection error!!!....................."<<endl;
		Sleep(5000);
		cvReleaseMat(&IR_object_points);
		cvReleaseMat(&IR_image_points);
		cvReleaseMat(&IR_point_counts);
		cvReleaseMat(&RGB_object_points);
		cvReleaseMat(&RGB_image_points);
		cvReleaseMat(&RGB_point_counts);
		cvReleaseMat(&RGB_intrinsic_matrix);
		cvReleaseMat(&RGB_distortion_coeffs);
		cvReleaseMat(&IR_intrinsic_matrix);
		cvReleaseMat(&IR_distortion_coeffs);
		delete [] RGB_corners;
		delete [] IR_corners;
		cvDestroyWindow("RGB frame");
		cvDestroyWindow("IR frame");
		cvDestroyWindow("calibrate RGB");
		cvDestroyWindow("calibrate IR");
		cvReleaseImage(&gray_image);
		cvReleaseImage(&colorframe);
		cvReleaseImage(&image);
		///shut down the nui
		NuiShutdown();
		return 0;
	}*/

	/////////////////////////////////////////////////////////////Collect data for stereo calibration (IR camera and RGB camera)//////////////////////////////////////////////////////////////////////////
	RGB_intrinsic_matrix=(CvMat*)cvLoad("calibrated_Intrinsic_RGB2.xml");
	RGB_distortion_coeffs0=(CvMat*)cvLoad("calibrated_Distortion_RGB2.xml");
	IR_intrinsic_matrix=(CvMat*)cvLoad("calibrated_Intrinsic_IR2.xml");
	IR_distortion_coeffs0=(CvMat*)cvLoad("calibrated_Distortion_IR2.xml");

	for(int i=0;i<4;i++){
		CV_MAT_ELEM(*RGB_distortion_coeffs,float,0,i)=CV_MAT_ELEM(*RGB_distortion_coeffs0,float,i,0);
	}
	CV_MAT_ELEM(*RGB_distortion_coeffs,float,0,4)=0;

	for(int i=0;i<4;i++){
		CV_MAT_ELEM(*IR_distortion_coeffs,float,0,i)=CV_MAT_ELEM(*IR_distortion_coeffs0,float,i,0);
	}
	CV_MAT_ELEM(*IR_distortion_coeffs,float,0,4)=0;

	CvMat *R1=cvCreateMat(3,3,CV_64FC1);
	CvMat *T1=cvCreateMat(3,1,CV_64FC1);
	CvMat *E1=cvCreateMat(3,3,CV_64FC1);
	CvMat *F1=cvCreateMat(3,3,CV_64FC1);
	//at this point, we have all of the chessboard corners we need(including IR and RGB).
	//calibrate the stereo of IR camera and RGB camera
	//note the function cvStereoCalibrate() is not defined in Opencv 1.0. It's only included in the cv.h of Opencv1.1 or a higher versionl.
	CvMat *sub_object_points=cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat *sub_point_counts=cvCreateMat(n_boards,1,CV_32FC1);
	cvSub(RGB_object_points,IR_object_points,sub_object_points);
	cvSub(RGB_point_counts,IR_point_counts,sub_point_counts);
	double rms=cvStereoCalibrate(
		RGB_object_points,
		RGB_image_points,
		IR_image_points,
		RGB_point_counts,
		RGB_intrinsic_matrix,
		RGB_distortion_coeffs,
		IR_intrinsic_matrix,
		IR_distortion_coeffs,
		cvGetSize(image),
		R1,
		T1,
		E1,
		F1,
		cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,100,1e-5),
		CV_CALIB_FIX_INTRINSIC);//不对两个相机的内参做细化
	cout << "done with RMS error=" << rms << endl;

	cvSave("R2_test1.xml",R1);
	cvSave("T2_test1.xml",T1);
	cvSave("E2_test1.xml",E1);
	cvSave("F2_test1.xml",F1);
	///////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	cvReleaseMat(&IR_object_points);
	cvReleaseMat(&IR_image_points);
	cvReleaseMat(&IR_point_counts);
	cvReleaseMat(&RGB_object_points);
	cvReleaseMat(&RGB_image_points);
	cvReleaseMat(&RGB_point_counts);
	cvReleaseMat(&RGB_intrinsic_matrix);
	cvReleaseMat(&RGB_distortion_coeffs);
	cvReleaseMat(&IR_intrinsic_matrix);
	cvReleaseMat(&IR_distortion_coeffs);
	delete [] RGB_corners;
	delete [] IR_corners;
	cvDestroyWindow("RGB frame");
	cvDestroyWindow("IR frame");
	cvDestroyWindow("calibrate RGB");
	cvDestroyWindow("calibrate IR");
	cvReleaseImage(&gray_image);
	cvReleaseImage(&colorframe);
	cvReleaseImage(&image);
	cvReleaseMat(&R1);
	cvReleaseMat(&T1);
	cvReleaseMat(&E1);
	cvReleaseMat(&F1);
	cvReleaseMat(&sub_object_points);
	cvReleaseMat(&sub_point_counts);
	///shut down the nui
	NuiShutdown();
	return 0;
}
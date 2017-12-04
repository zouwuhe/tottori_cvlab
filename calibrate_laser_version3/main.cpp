//the purpose of this project is to calibrate between the laser and the chessboard
//click the "chessboard" to choose the laser source, then press the "enter" to continue the calibration
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <cxcore.h>
#include <NuiApi.h>
#include <iostream>
#include <math.h>

using namespace std;

int n_boards=0;
const int board_dt=20;//wait 20 frames per chessboard view
int board_w;//the width of chessboard
int board_h;//the highth of chessboard
long angle;//the anlge of kinect1
bool IS_data_effective=false;

int main(int argc,char* argv[]){
	char *filename="n_laser_total.xml";
	cout<<"Input these parameters:"<<endl;
	cout<<"1. the width of chessboard"<<endl;
	cout<<"2. the highth of chessboard"<<endl;
	cout<<"3. the total number of images used for calibrating laser"<<endl;
	cout<<"4. the angle of kinect1"<<endl;
	cin>>board_w>>board_h>>n_boards>>angle;
	//read in the data from the RGB camera of kinect1 //先启动 kinect1，并打开kinect1数据流，然后读取RGB图像
	cvNamedWindow("colorframe");
	cvNamedWindow("calibration");
	IplImage *colorframe=cvCreateImage(cvSize(640,480),8,3);
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
		NUI_IMAGE_RESOLUTION_640x480,
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
	//allocate storage
	CvMat *image_points1=cvCreateMat(board_n,2,CV_32FC1);
	CvMat *object_points1=cvCreateMat(board_n,3,CV_32FC1);
	CvMat *image_points2=cvCreateMat(board_n,3,CV_32FC1);//used for calculate homography
	CvMat *object_points2=cvCreateMat(board_n,3,CV_32FC1);
	CvMat *intrinsic=(CvMat*)cvLoad("Intrinsic1.xml");
	CvMat *distortion=(CvMat*)cvLoad("Distortion1.xml");
	CvPoint2D32f *corners=new CvPoint2D32f[board_n];
	CvMat *Hd=cvCreateMat(3,3,CV_32FC1);
	CvMat *rotation_vector=cvCreateMat(3,1,CV_32FC1);
	CvMat *rotation_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *rotation_matrix_invert=cvCreateMat(3,3,CV_32FC1);
	CvMat *rotation_matrix_total=cvCreateMat(3,3*n_boards,CV_32FC1);
	CvMat *translation_vector=cvCreateMat(3,1,CV_32FC1);
	CvMat *laser_source=cvCreateMat(3,1,CV_32FC1);//the laser source point in image
	CvMat *Po_total=(CvMat*)cvLoad("Po.xml");
	CvMat *Po=cvCreateMat(3,1,CV_32FC1);//the laser source point in object coordinate system
	CV_MAT_ELEM(*Po,float,0,0)=CV_MAT_ELEM(*Po_total,float,0,0);
	CV_MAT_ELEM(*Po,float,1,0)=CV_MAT_ELEM(*Po_total,float,1,0);
	CV_MAT_ELEM(*Po,float,2,0)=0;
	CvMat *Po_show=cvCreateMat(3,1,CV_32FC1);//用于显示P点位置
	CV_MAT_ELEM(*Po_show,float,0,0)=CV_MAT_ELEM(*Po_total,float,0,0);
	CV_MAT_ELEM(*Po_show,float,1,0)=CV_MAT_ELEM(*Po_total,float,1,0);
	CV_MAT_ELEM(*Po_show,float,2,0)=1.0f;
	CvMat *Pc=cvCreateMat(3,1,CV_32FC1);//the laser source point in camera coordinate system
	CvMat *n_laser1=cvCreateMat(3,1,CV_32FC1);
	CvMat *n_laser2=cvCreateMat(3,1,CV_32FC1);
	CvMat *n_laser_total=cvCreateMat(3,n_boards,CV_32FC1);
	int corner_count;
	int successes=0;
	int frame=0;
	IplImage *image=cvCreateImage(cvSize(640,480),8,3);
	IplImage *gray_image=cvCreateImage(cvSize(640,480),8,1);
	/////////////main loop
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
				cvShowImage("colorframe",colorframe);
				cvWaitKey(33);//需要给一个延时才能同步显示图像，因为显示图像需要时间
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
			cvShowImage("calibration",image);
			cvWaitKey(33);//需要给一个延时才能同步显示图像，因为显示图像需要时间
			//if we got a good board, calculate the Hd(单映性矩阵) and R,T(棋盘位置);
			//then, repeat showing the board and wait for clicking the laser source
			if(corner_count==board_n){
				for(int i=0;i<board_n;i++){
					CV_MAT_ELEM(*image_points1,float,i,0)=corners[i].x;
					CV_MAT_ELEM(*image_points1,float,i,1)=image->height-corners[i].y;
					CV_MAT_ELEM(*object_points1,float,i,0)=(float)(i/board_w);
					CV_MAT_ELEM(*object_points1,float,i,1)=(float)(i%board_w);
					CV_MAT_ELEM(*object_points1,float,i,2)=0.0f;
					//////////////////////////////////////////////////
					CV_MAT_ELEM(*image_points2,float,i,0)=corners[i].x;
					CV_MAT_ELEM(*image_points2,float,i,1)=image->height-corners[i].y;
					CV_MAT_ELEM(*image_points2,float,i,2)=1.0f;
					CV_MAT_ELEM(*object_points2,float,i,0)=(float)(i/board_w);
					CV_MAT_ELEM(*object_points2,float,i,1)=(float)(i%board_w);
					CV_MAT_ELEM(*object_points2,float,i,2)=1.0f;
				}
				cvFindHomography(object_points2,image_points2,Hd);
				//draw P in the chessboard
				cvMatMul(Hd,Po_show,laser_source);
				CV_MAT_ELEM(*laser_source,float,0,0)=CV_MAT_ELEM(*laser_source,float,0,0)/CV_MAT_ELEM(*laser_source,float,2,0);//由于laser_source本该为其次表达式，因此重新其次化以减小误差
				CV_MAT_ELEM(*laser_source,float,1,0)=image->height-CV_MAT_ELEM(*laser_source,float,1,0)/CV_MAT_ELEM(*laser_source,float,2,0);
				CV_MAT_ELEM(*laser_source,float,2,0)=0;//为了便于求解将其次坐标与单应性矩阵相对应，因此取laser_source的前两项为图像上的laser source的x，y坐标。
				cvCircle(image,cvPoint(CV_MAT_ELEM(*laser_source,float,0,0),CV_MAT_ELEM(*laser_source,float,1,0)),2,cvScalar(0,0,255),2,8,0);
				//judge whether use these data or not
				while(1){
					cvShowImage("calibration",image);
					char c=cvWaitKey(33);
					if(c==13){
						IS_data_effective=true;
						break;
					}
					if(c=='p'){
						IS_data_effective=false;
						break;
					}
				}
				//if press 'p',we will pass these data and turn to next image;
				//if press 'enter', we will calculate the n_laser2 and add to the n_laser_total; also, the number of effective data plus 1(successes++)
				if(IS_data_effective){
					cvFindExtrinsicCameraParams2(object_points1,image_points1,intrinsic,distortion,rotation_vector,translation_vector);
					cvRodrigues2(rotation_vector,rotation_matrix);//calculate the rotation matrix from rotation vector
					cvMatMulAdd(rotation_matrix,Po,translation_vector,Pc);
					//calculate the n_laser1
					CV_MAT_ELEM(*n_laser1,float,0,0)=-CV_MAT_ELEM(*Pc,float,0,0);
					CV_MAT_ELEM(*n_laser1,float,1,0)=-CV_MAT_ELEM(*Pc,float,1,0);
					CV_MAT_ELEM(*n_laser1,float,2,0)=-CV_MAT_ELEM(*Pc,float,2,0);
					//calculate the n_laser2
					cvInvert(rotation_matrix,rotation_matrix_invert);//默认高斯消去法求逆矩阵
					cvMatMul(rotation_matrix_invert,n_laser1,n_laser2);
					//将 n_laser2 单位化并输出
					cvNormalize(n_laser2,n_laser2);//normalize
					cout<<"第"<<successes<<"个向量n_laser2:"<<endl;
					for(int i=0;i<3;i++){
						cout<<CV_MAT_ELEM(*n_laser2,float,i,0)<<"  ";
					}
					cout<<endl;
					//store the n_laser2 to n_laser_total
					CV_MAT_ELEM(*n_laser_total,float,0,successes)=CV_MAT_ELEM(*n_laser2,float,0,0);
					CV_MAT_ELEM(*n_laser_total,float,1,successes)=CV_MAT_ELEM(*n_laser2,float,1,0);
					CV_MAT_ELEM(*n_laser_total,float,2,successes)=CV_MAT_ELEM(*n_laser2,float,2,0);//保存每一次校正出来的激光相对于chessboard的方向n_laser2
					successes++;
				}
			}
		}
		if(successes>=n_boards){
			cvSave(filename,n_laser_total);
			break;
		}
		if(cvWaitKey(10)==27) break;
	}
	delete [] corners;
	cvReleaseMat(&object_points1);
	cvReleaseMat(&image_points1);
	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&intrinsic);
	cvReleaseMat(&distortion);
	cvReleaseMat(&Hd);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&rotation_matrix);
	cvReleaseMat(&rotation_matrix_total);
	cvReleaseMat(&rotation_matrix_invert);
	cvReleaseMat(&translation_vector);
	cvReleaseMat(&laser_source);
	cvReleaseMat(&Po);
	cvReleaseMat(&Po_show);
	cvReleaseMat(&Po_total);
	cvReleaseMat(&Pc);
	cvReleaseMat(&n_laser1);
	cvReleaseMat(&n_laser2);
	cvReleaseMat(&n_laser_total);
	cvReleaseImage(&colorframe);
	cvReleaseImage(&image);
	cvReleaseImage(&gray_image);
	cvDestroyWindow("colorframe");
	cvDestroyWindow("calibration");
	///shut down the nui
	NuiShutdown();
	return 0;
}
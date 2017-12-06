#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <NuiApi.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace std;
void calibrate_2kinects_version1(string filename);


int test_num=0;
const int board_dt=30;//wait 80 frames per chessboard view
int board_w;//the width of chessboard
int board_h;//the highth of chessboard
long angle1;//the anlge of kinect1
long angle2;//the angle of kinect2
double chessboard_width=0.026;//定义chessboard中每个方格的边长，作为计算Pc的实际坐标时候的换算单位
bool IS_data_effective=false;

int main(){
	string filename="experiment_test11_1_2.txt";//
	ofstream outfile(filename);
	//input the parameters
	cout<<"Input these parameters:"<<endl;
	cout<<"1. the width of chessboard"<<endl;
	cout<<"2. the highth of chessboard"<<endl;
	cout<<"3. the total number of images used for calibrating pose between kinects"<<endl;
	cout<<"4. the angle of kinect1"<<endl;
	cout<<"5. the angle of kinect2"<<endl;
	cin>>board_w>>board_h>>test_num>>angle1>>angle2;
	outfile<<test_num<<endl;// output to file
	//outfile<<"test number:"<<test_num<<endl;// output to file
	//////////
	cvNamedWindow("colorframe1");
	cvNamedWindow("colorframe2");
	IplImage *colorframe1=cvCreateImage(cvSize(640,480),8,3);
	IplImage *colorframe2=cvCreateImage(cvSize(640,480),8,3);
	///build two Kinect sensor
	//Kinect1对应m_pNuiSensor,Kinect2对应n_pNuiSensor
	//kinect1
	INuiSensor *m_pNuiSensor;
	BSTR m_instanceid;
	HRESULT hr;
	hr=NuiCreateSensorByIndex(0,&m_pNuiSensor);
	m_instanceid=m_pNuiSensor->NuiDeviceConnectionId();
	//kinect2
	INuiSensor *n_pNuiSensor;
	BSTR n_instanceid;
	hr=NuiCreateSensorByIndex(1,&n_pNuiSensor);
	n_instanceid=n_pNuiSensor->NuiDeviceConnectionId();
	///initialize
	//kinect1
	hr=m_pNuiSensor->NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_DEPTH|NUI_INITIALIZE_FLAG_USES_SKELETON);///NUI_INITIALIZE_FLAG_USES_DEPTH can be changed to NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
	if(hr!=S_OK){
		cout<<"Kinect1 NuiInitialize failed\n"<<endl;
		return hr;
	}
	//adjust the angle of the kinect1
	m_pNuiSensor->NuiCameraElevationSetAngle(angle1);
	//kinect2
	hr=n_pNuiSensor->NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_DEPTH|NUI_INITIALIZE_FLAG_USES_SKELETON);///NUI_INITIALIZE_FLAG_USES_DEPTH can be changed to NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
	if(hr!=S_OK){
		cout<<"Kinect2 NuiInitialize failed\n"<<endl;
		return hr;
	}
	//adjust the angle of the kinect2
	n_pNuiSensor->NuiCameraElevationSetAngle(angle2);
	///open the color frame stream
	//kinect1
	HANDLE m_hNextColorFrameEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	HANDLE m_pColorStreamHandle=NULL;
	hr=m_pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		m_hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
		&m_pColorStreamHandle);//打开的彩色数据流的句柄的地址
	if(FAILED(hr)){
		cout<<"Can't open the color frame stream of Kinect1\n"<<endl;
		return hr;
	}
	//kinect2
	HANDLE n_hNextColorFrameEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	HANDLE n_pColorStreamHandle=NULL;
	hr=n_pNuiSensor->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		n_hNextColorFrameEvent,//事件句柄，下一帧彩色数据准备好时，该事件句柄被激活
		&n_pColorStreamHandle);//打开的彩色数据流的句柄的地址
	if(FAILED(hr)){
		cout<<"Can't open the color frame stream of Kinect2\n"<<endl;
		return hr;
	}
	///configure events to be listened on
	HANDLE hEvents[2];
	hEvents[0]=m_hNextColorFrameEvent;
	hEvents[1]=n_hNextColorFrameEvent;
	////////////////////////////////////////////
	//prepare for the collection of chessboard
	int board_n=board_w*board_h;
	CvSize board_sz=cvSize(board_w,board_h);
	//allocate storage
	CvMat *image_points1=cvCreateMat(board_n,2,CV_32FC1);
	CvMat *object_points1=cvCreateMat(board_n,3,CV_32FC1);
	CvMat *image_points2=cvCreateMat(board_n,3,CV_32FC1);//used for calculate homography
	CvMat *object_points2=cvCreateMat(board_n,3,CV_32FC1);
	CvMat *intrinsic1=(CvMat*)cvLoad("Intrinsic1.xml");//参数对应：用kinect1观察chessboard，用kienct2观察background
	CvMat *distortion1=(CvMat*)cvLoad("Distortion1.xml");
	CvMat *intrinsic2=(CvMat*)cvLoad("Intrinsic2.xml");
	double fx=CV_MAT_ELEM(*intrinsic2,float,0,0);
	double fy=CV_MAT_ELEM(*intrinsic2,float,1,1);
	double cx=CV_MAT_ELEM(*intrinsic2,float,0,2);
	double cy=CV_MAT_ELEM(*intrinsic2,float,1,2);
	outfile<<fx<<' '<<fy<<' '<<cx<<' '<<cy<<endl;//output to file// the fx,fy,cx,cy is belong to kinect2
	//outfile<<"Kinect2 constant parameter fx,fy,cx,cy:"<<fx<<' '<<fy<<' '<<cx<<' '<<cy<<endl;//output to file// the fx,fy,cx,cy is belong to kinect2
	//outfile<<"Observed quantity qa,qb,n1,n2,n3,Tpx,Tpy,Tpz:"<<endl;
	CvPoint2D32f *corners=new CvPoint2D32f[board_n];
	CvMat *Hd=cvCreateMat(3,3,CV_32FC1);
	CvMat *rotation_vector=cvCreateMat(3,1,CV_32FC1);
	CvMat *rotation_matrix=cvCreateMat(3,3,CV_32FC1);
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
	CvMat *n_laser_total=(CvMat*)cvLoad("n_laser_total8.xml");
	CvMat *n_laser1=cvCreateMat(3,1,CV_32FC1);
	CvMat *n_laser2=cvCreateMat(3,1,CV_32FC1);
	CV_MAT_ELEM(*n_laser1,float,0,0)=CV_MAT_ELEM(*n_laser_total,float,0,0);
	CV_MAT_ELEM(*n_laser1,float,1,0)=CV_MAT_ELEM(*n_laser_total,float,1,0);
	CV_MAT_ELEM(*n_laser1,float,2,0)=CV_MAT_ELEM(*n_laser_total,float,2,0);//n_laser1 is the calibrated laser
	/*//////////////////////////////////////////n_laser1单位化
	double s=sqrt(pow((double)CV_MAT_ELEM(*n_laser1,float,0,0),2.0)+pow((double)CV_MAT_ELEM(*n_laser1,float,1,0),2.0)+pow((double)CV_MAT_ELEM(*n_laser1,float,2,0),2.0));
	CV_MAT_ELEM(*n_laser1,float,0,0)=CV_MAT_ELEM(*n_laser1,float,0,0)/s;
	CV_MAT_ELEM(*n_laser1,float,1,0)=CV_MAT_ELEM(*n_laser1,float,0,0)/s;
	CV_MAT_ELEM(*n_laser1,float,2,0)=CV_MAT_ELEM(*n_laser1,float,0,0)/s;
	//////////////////////////////////////////*///实验证明单位化对校正没有帮助，反而有不好的影响，容易导致结果发散，可能是单位化的过程引入了误差
	CvPoint laserpoint_q=cvPoint(0,0);//the laserpoint in colorframe2
	int corner_count;
	int successes=0;
	int frame=0;
	IplImage *image=cvCreateImage(cvSize(640,480),8,3);
	IplImage *gray_image=cvCreateImage(cvSize(640,480),8,1);
	IplImage *original_image=cvCreateImage(cvSize(640,480),8,3);
	IplImage *difference_image=cvCreateImage(cvSize(640,480),8,1);
	/////////////main loop
	while(1){
		WaitForMultipleObjects(sizeof(hEvents)/sizeof(hEvents[0]),hEvents,FALSE,100);//?
		if(WAIT_OBJECT_0==WaitForSingleObject(m_hNextColorFrameEvent,0)){
			///read the color frame from color data stream of kinect1
			const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for color data
			hr=NuiImageStreamGetNextFrame(
				m_pColorStreamHandle,
				0,//不需继续等待新图像帧而直接返回的时间
				&pImageFrame);//从指定数据流获取下一帧信息，信息地址存放在NUI_IMAGE_FRAME的结构体指针，结构体中的INuiFrameTexture包含帧数据
			if(FAILED(hr)){
				cout<<"Get color frame from Kinect1 failed\n"<<endl;
				continue;
			}
			INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //从NUI_IMAGE_FRAME结构体中，提取出需要的帧数据
			NUI_LOCKED_RECT lockedrect;//定义一个结构体，用于保存锁定区域的信息
			pTexture->LockRect(0,&lockedrect,NULL,0);
			if(lockedrect.Pitch!=0){//pitch返回缓冲区中一行的字节数
				BYTE *pBuffer=(BYTE*)lockedrect.pBits;//缓冲区首地址
				for(int y=0;y<480;y++){
					uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
						colorframe1->imageData+y*colorframe1->widthStep
						);
					for(int x=0;x<640;x++){
						for(int z=0;z<3;z++){
							color_ptr[3*x+z]=pBuffer[y*4*640+x*4+z];
						}
					}
				}
				cvShowImage("colorframe1",colorframe1);
			}
			else{
				cout<<"Bufferlength of received texture from color buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(m_pColorStreamHandle,pImageFrame);
		}
		if(WAIT_OBJECT_0==WaitForSingleObject(n_hNextColorFrameEvent,0)){
			///read the color frame from color data stream of kinect2
			const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for color data
			hr=NuiImageStreamGetNextFrame(
				n_pColorStreamHandle,
				0,//不需继续等待新图像帧而直接返回的时间
				&pImageFrame);//从指定数据流获取下一帧信息，信息地址存放在NUI_IMAGE_FRAME的结构体指针，结构体中的INuiFrameTexture包含帧数据
			if(FAILED(hr)){
				cout<<"Get color frame from Kinect2 failed\n"<<endl;
				continue;
			}
			INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //从NUI_IMAGE_FRAME结构体中，提取出需要的帧数据
			NUI_LOCKED_RECT lockedrect;//定义一个结构体，用于保存锁定区域的信息
			pTexture->LockRect(0,&lockedrect,NULL,0);
			if(lockedrect.Pitch!=0){//pitch返回缓冲区中一行的字节数
				BYTE *pBuffer=(BYTE*)lockedrect.pBits;//缓冲区首地址
				for(int y=0;y<480;y++){
					uchar* color_ptr=(uchar*)(//彩色数据为uchar类型
						colorframe2->imageData+y*colorframe2->widthStep
						);
					for(int x=0;x<640;x++){
						for(int z=0;z<3;z++){
							color_ptr[3*x+z]=pBuffer[y*4*640+x*4+z];
						}
					}
				}
				cvShowImage("colorframe2",colorframe2);
			}
			else{
				cout<<"Bufferlength of received texture from color buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(n_pColorStreamHandle,pImageFrame);
		}
		if(cvWaitKey(1)=='y'){
			cvCopyImage(colorframe2,original_image);//如果按下'y'，就将此时的colorframe2保存下来作为原始图像，留待差分找laser point用。
			cout<<"original image refreshed!"<<endl;
		}
		//collection of chessboard
		if(frame++%board_dt==0){//每间隔board_dt帧尝试一次 采集图像上的棋盘角点
			cvCopyImage(colorframe1,image);
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
			cvShowImage("colorframe1",image);
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
					CV_MAT_ELEM(*object_points2,float,i,1)=(float)(i%board_w);//注意，此处将baord_w定位y
					CV_MAT_ELEM(*object_points2,float,i,2)=1.0f;
				}
				cvFindHomography(object_points2,image_points2,Hd);
				//draw P in the chessboard
				cvMatMul(Hd,Po_show,laser_source);
				CV_MAT_ELEM(*laser_source,float,0,0)=CV_MAT_ELEM(*laser_source,float,0,0)/CV_MAT_ELEM(*laser_source,float,2,0);//由于laser_source本该为其次表达式，因此重新其次化以减小误差
				CV_MAT_ELEM(*laser_source,float,1,0)=image->height-CV_MAT_ELEM(*laser_source,float,1,0)/CV_MAT_ELEM(*laser_source,float,2,0);
				CV_MAT_ELEM(*laser_source,float,2,0)=0;//为了便于求解将其次坐标与单应性矩阵相对应，因此取laser_source的前两项为图像上的laser source的x，y坐标。
				cvCircle(image,cvPoint((int)CV_MAT_ELEM(*laser_source,float,0,0),(int)CV_MAT_ELEM(*laser_source,float,1,0)),2,cvScalar(0,0,255),2,8,0);
				////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				//calculate the position of laser point with difference between the original colorframe2 and present colorframe2
				double max_difference=0;
				for(int y=0;y<difference_image->height;y++){
					uchar* ptr_difference=(uchar*)(
						difference_image->imageData+y*difference_image->widthStep
						);
					uchar* ptr_colorframe2=(uchar*)(
						colorframe2->imageData+y*colorframe2->widthStep
						);
					uchar* ptr_original=(uchar*)(
						original_image->imageData+y*original_image->widthStep
						);
					for(int x=0;x<difference_image->width;x++){
						ptr_difference[x]=(uchar)sqrt(pow(ptr_colorframe2[3*x]-ptr_original[3*x],2.0)+pow(ptr_colorframe2[3*x+1]-ptr_original[3*x+1],2.0)+pow(ptr_colorframe2[3*x+2]-ptr_original[3*x+2],2.0));
						if(ptr_difference[x]>max_difference){
							max_difference=ptr_difference[x];
							laserpoint_q=cvPoint(x,y);
						}
					}
				}
				cvCircle(colorframe2,laserpoint_q,2,cvScalar(0,255,0),2,8,0);
				//////////////////////////////////////////////////////////////////////////////////////////////////////////////
				/*//judge whether use these data or not
				while(1){
					cvShowImage("colorframe1",image);
					cvShowImage("colorframe2",colorframe2);
					char c=cvWaitKey(33);
					if(c==13){
						IS_data_effective=true;
						break;
					}
					if(c=='p'){
						IS_data_effective=false;
						break;
					}
				}*/
				//if press 'p',we will pass these data and turn to next image;
				//if press 'enter', we will calculate and add the data; also, the number of effective data plus 1(successes++)
				/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
				cvShowImage("colorframe1",image);
				cvShowImage("colorframe2",colorframe2);
				char c=0;
				c=cvWaitKey(1000);
				if(c=='p'){
					IS_data_effective=false;
					cout<<"get rid of error!"<<endl;
				}
				else{
					IS_data_effective=true;
				}
				///////显示检测结果并等待2s，期间发现问题则按下"p"，然后自动剔除这组数据并显示"get rid of error!"；否则直接采用测量数据。
				if(IS_data_effective){
					cvFindExtrinsicCameraParams2(object_points1,image_points1,intrinsic1,distortion1,rotation_vector,translation_vector);
					cvRodrigues2(rotation_vector,rotation_matrix);//calculate the rotation matrix from rotation vector
					////////
					cvMatMulAdd(rotation_matrix,Po,translation_vector,Pc);//Pc is equal to Tp (Tp)
					//calculate the n_laser2
					cvMatMul(rotation_matrix,n_laser1,n_laser2);//n_laser1 has been calibrated before and stored, and n_laser2 is the laser right now (n)
					//Pc,n_laser2,laserpoint_q is the data we need
					//laserpoint_q 为Q在kinect2上的成像点
					outfile<<laserpoint_q.x<<' '<<image->height-laserpoint_q.y<<' '<<CV_MAT_ELEM(*n_laser2,float,0,0)<<' '<<CV_MAT_ELEM(*n_laser2,float,1,0)
						<<' '<<CV_MAT_ELEM(*n_laser2,float,2,0)<<' '<<CV_MAT_ELEM(*Pc,float,0,0)*chessboard_width<<' '<<CV_MAT_ELEM(*Pc,float,1,0)*chessboard_width<<' '<<CV_MAT_ELEM(*Pc,float,2,0)*chessboard_width<<endl;
					//将Pc的坐标以m为单位保存,q的坐标以对应一般中心投影的坐标保存
					successes++;
				}
			}
		}
		if(successes>=test_num){//complete the collection of data
			break;
		}
		if(cvWaitKey(10)==27) break;
	}
	outfile<<0<<' '<<0<<' '<<0<<' '<<1<<' '<<0<<' '<<0<<' '<<0<<endl;
	//close the output file
	outfile.close();
	delete [] corners;
	cvReleaseMat(&object_points1);
	cvReleaseMat(&image_points1);
	cvReleaseMat(&object_points2);
	cvReleaseMat(&image_points2);
	cvReleaseMat(&intrinsic1);
	cvReleaseMat(&distortion1);
	cvReleaseMat(&intrinsic2);
	cvReleaseMat(&Hd);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&rotation_matrix);
	cvReleaseMat(&translation_vector);
	cvReleaseMat(&laser_source);
	cvReleaseMat(&Po);
	cvReleaseMat(&Po_total);
	cvReleaseMat(&Po_show);
	cvReleaseMat(&Pc);
	cvReleaseMat(&n_laser1);
	cvReleaseMat(&n_laser2);
	cvReleaseMat(&n_laser_total);
	cvReleaseImage(&colorframe1);
	cvReleaseImage(&colorframe2);
	cvReleaseImage(&image);
	cvReleaseImage(&gray_image);
	cvReleaseImage(&difference_image);
	cvReleaseImage(&original_image);
	cvDestroyWindow("colorframe1");
	cvDestroyWindow("colorframe2");
	///shut down the nui
	m_pNuiSensor->NuiShutdown();
	n_pNuiSensor->NuiShutdown();
	//calibrate based on the stored data.........//也可以考虑显示出每一组数据对应的激光光斑，并标出对应的epipole误差
	calibrate_2kinects_version1(filename);
	return 0;
}
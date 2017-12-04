#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <cxcore.h>
#include <cvcam.h>
#include <NuiApi.h>
#include <iostream>
#include <vector>

using namespace std;

struct Point3D{// the struct of 3D point of depth image
	int x;
	int y;
	ushort depth;
};//一个点在kinect的depth camera 下的坐标以及深度值

double evaluate_depth_accuracy(int board_dt,int board_w,int board_h,HANDLE hNextColorFrameEvent,HANDLE pColorStreamHandle,HANDLE hNextDepthFrameEvent,HANDLE pDepthStreamHandle,double c0,double c1){
	int board_n=board_w*board_h;
	CvSize board_sz=cvSize(board_w,board_h);
	int frame=0;
	int corner_count;
	bool IR_successes=0;
	HRESULT hr;
	IplImage *colorframe=cvCreateImage(cvSize(640,480),8,3);
	IplImage *image=cvCreateImage(cvSize(640,480),8,3);
	IplImage *gray_image=cvCreateImage(cvSize(640,480),8,1);
	IplImage *depthframe=cvCreateImage(cvSize(640,480),16,1);
	IplImage *depth_image=cvCreateImage(cvSize(640,480),16,1);
	//allocate storage for calibrate depth camera
	CvMat *IR_image_points1=cvCreateMat(board_n,2,CV_32FC1);//used for calculate R,T of pattern
	CvMat *IR_object_points1=cvCreateMat(board_n,3,CV_32FC1);
	CvMat *IR_image_points2=cvCreateMat(board_n,3,CV_32FC1);//used for calculate homography
	CvMat *IR_object_points2=cvCreateMat(board_n,3,CV_32FC1);
	CvMat *IR_intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *IR_distortion_coeffs=cvCreateMat(4,1,CV_32FC1);
	CvPoint2D32f *IR_corners=new CvPoint2D32f[board_n];
	CvMat *Hd=cvCreateMat(3,3,CV_32FC1);/////////////////homography
	CvMat *rotation_vector=cvCreateMat(3,1,CV_32FC1);
	CvMat *rotation_matrix=cvCreateMat(3,3,CV_32FC1);/////////////////used for calculate depth accuracy
	CvMat *translation_vector=cvCreateMat(3,1,CV_32FC1);/////////////////used for calculate depth accuracy

	//load these matrices back in
	IR_intrinsic_matrix=(CvMat*)cvLoad("calibrated_Intrinsic_IR2.xml");
	IR_distortion_coeffs=(CvMat*)cvLoad("calibrated_Distortion_IR2.xml");
	/////////////main loop for calculate the pattern of IR image
	while(!IR_successes){
		//read in the IR image in real time.
		if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
			///read the IR frame from color data stream
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
				cout<<"Bufferlength of received texture from infrared buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(pColorStreamHandle,pImageFrame);
		}
		cvCopyImage(colorframe,image);
		//collection of chessboard
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
			cvShowImage("IR pattern",image);
			cvWaitKey(33);//显示图像是需要时间的，所以必须给一个延时，否则无法同步看到图像
			//if we got a good board, use this pattern to calculate the R,T
			if(corner_count==board_n){
				for(int i=0;i<board_n;++i){
					CV_MAT_ELEM(*IR_image_points1,float,i,0)=IR_corners[i].x;
					CV_MAT_ELEM(*IR_image_points1,float,i,1)=image->height-IR_corners[i].y;
					CV_MAT_ELEM(*IR_object_points1,float,i,0)=i/board_w;
					CV_MAT_ELEM(*IR_object_points1,float,i,1)=i%board_w;
					CV_MAT_ELEM(*IR_object_points1,float,i,2)=0.0f;
					//////////////////////////////////////////////////
					CV_MAT_ELEM(*IR_image_points2,float,i,0)=IR_corners[i].x;
					CV_MAT_ELEM(*IR_image_points2,float,i,1)=image->height-IR_corners[i].y;
					CV_MAT_ELEM(*IR_image_points2,float,i,2)=1.0f;
					CV_MAT_ELEM(*IR_object_points2,float,i,0)=(float)(i/board_w);
					CV_MAT_ELEM(*IR_object_points2,float,i,1)=(float)(i%board_w);//注意，此处将baord_w定为y
					CV_MAT_ELEM(*IR_object_points2,float,i,2)=1.0f;
				}
				cvFindHomography(IR_image_points2,IR_object_points2,Hd);//计算单应性矩阵，从图像点对应到pattern上的坐标点
				cvFindExtrinsicCameraParams2(IR_object_points1,IR_image_points1,IR_intrinsic_matrix,IR_distortion_coeffs,rotation_vector,translation_vector);
				cvRodrigues2(rotation_vector,rotation_matrix);//calculate the rotation matrix from rotation vector
				IR_successes=true;
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
			delete [] IR_corners;
			cvReleaseMat(&IR_object_points1);
			cvReleaseMat(&IR_image_points1);
			cvReleaseMat(&IR_object_points2);
			cvReleaseMat(&IR_image_points2);
			cvReleaseMat(&IR_intrinsic_matrix);
			cvReleaseMat(&IR_distortion_coeffs);
			cvReleaseMat(&rotation_vector);
			cvReleaseImage(&gray_image);
			cvReleaseImage(&colorframe);
			cvReleaseImage(&depthframe);
			cvReleaseImage(&depth_image);
			cvReleaseImage(&image);
			NuiShutdown();
			return 0;
		}
	}
	////////////////////////////////////////
	///////////////////构造四边形用于选取标定板内部的点
	//float value_fVertex=0;
	//float* pfVertex=&value_fVertex; //多边形顶点位置
	float pfVertex[8];
	//.....为pfVertex赋值
	pfVertex[0]=IR_corners[0].x;
	pfVertex[1]=IR_corners[0].y;
	pfVertex[2]=IR_corners[board_w-1].x;
	pfVertex[3]=IR_corners[board_w-1].y;
	pfVertex[4]=IR_corners[board_n-board_w].x;
	pfVertex[5]=IR_corners[board_n-board_w].y;
	pfVertex[6]=IR_corners[board_n-1].x;
	pfVertex[7]=IR_corners[board_n-1].y;
	int ContourNum=4;  //多边形顶点数量
	CvMat* pmatContour=cvCreateMat(1,ContourNum,CV_32FC2);    // pmatContour是一个2通道的一维数组，两个通道  /////////////////用于确定用来计算深度精度的点的范围(限定在标定板内)
	for (int i=0;i<ContourNum;i++)
	{
		CvScalar vertex = cvScalar(pfVertex[2*i], pfVertex[2*i+1]);       // 一个二维顶点
		cvSet1D(pmatContour,i,vertex);                // 使用CvScalar作为数组元素
	}
	//pmatContour 即为四边形轮廓，用于后续检测点是否在标定板区域内
	//////////////////////////////////////// 画出四边形区域
	/*CvPoint value2_board_area=cvPoint(0,0);
	CvPoint *board_area=&value2_board_area;*/
	CvPoint board_area[4];
	board_area[0]=cvPoint((int)IR_corners[0].x,(int)IR_corners[0].y);
	board_area[1]=cvPoint((int)IR_corners[board_w-1].x,(int)IR_corners[board_w-1].y);
	board_area[2]=cvPoint((int)IR_corners[board_n-board_w].x,(int)IR_corners[board_n-board_w].y);
	board_area[3]=cvPoint((int)IR_corners[board_n-1].x,(int)IR_corners[board_n-1].y);
	cvFillConvexPoly(colorframe,board_area,4,CV_RGB(255,255,255));

	////////////////////////////////////////
	delete [] IR_corners;
	cvReleaseMat(&IR_object_points1);
	cvReleaseMat(&IR_image_points1);
	cvReleaseMat(&IR_object_points2);
	cvReleaseMat(&IR_image_points2);
	cvReleaseMat(&IR_intrinsic_matrix);
	cvReleaseMat(&IR_distortion_coeffs);
	cvReleaseMat(&rotation_vector);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////capture the depth image//////////////////////////////////////////////////////////
	while(1){
		if(WAIT_OBJECT_0==WaitForSingleObject(hNextDepthFrameEvent,0)){
			///read the depth frame from depth data stream
			const NUI_IMAGE_FRAME *pImageFrame=NULL;  //buffer for depth data
			hr=NuiImageStreamGetNextFrame(
				pDepthStreamHandle,
				0,
				&pImageFrame);
			if(FAILED(hr)){
				cout<<"Get depth frame failed\n"<<endl;
				continue;
			}
			INuiFrameTexture *pTexture=pImageFrame->pFrameTexture;
			NUI_LOCKED_RECT lockedrect;
			pTexture->LockRect(0,&lockedrect,NULL,0);
			if(lockedrect.Pitch!=0){
				BYTE *pBuffer=(BYTE*)lockedrect.pBits;
				for(int y=0;y<480;y++){
					ushort* depth_ptr=(ushort*)(//深度数据为ushort类型
						depthframe->imageData+y*depthframe->widthStep
						);
					uchar *pBufferRun=(uchar*)(lockedrect.pBits)+y*lockedrect.Pitch;//pitch返回的是缓冲区一行的字节数
					ushort*pBuffer=(ushort*)pBufferRun;
					for(int x=0;x<640;x++){
						depth_ptr[x]=pBuffer[x];
						//////////ptr[j] = 255 - (uchar)(256 * pBuffer[j]/0x0fff);  //也可以直接将数据归一化处 结果为char型 8bit深度图
					}
				}
				/*//显示x160y120位置上的像素信息
				ushort* depth_ptr=(ushort*)(//深度数据为ushort类型
						depthframe->imageData+120*depthframe->widthStep
						);//此时深度图像为ushort数据格式
				printf("x:160 y:120坐标处的深度值：d=%d\n",depth_ptr[160]);*/
				cvShowImage("depth frame",depthframe);
				cvWaitKey(33);
			}
			else{
				cout<<"Bufferlength of received texture from depth buffer is bogus\r\n"<<endl;
			}
			///release the data of this frame and be ready for next frame in the color stream
			NuiImageStreamReleaseFrame(pDepthStreamHandle,pImageFrame);
			/*if(lockedrect.Pitch!=0){
			ushort*pBuffer=(ushort*)((uchar*)(lockedrect.pBits)+119*lockedrect.Pitch);//提取深度数据缓存区域中的深度像素值，并以ushort的形式存在于pBuffer（16bit，低三位为player索引）
			//显示x160y120位置上的像素信息
			printf("x:160 y:120坐标处的深度值：d=%d\n",pBuffer[160]);//深度数据为ushort数据类型，因此数据范围0～65535,16bit 直接用%d 以十进制形式输出 由于低3位为0，因此为8的倍数
			}*/
		}
		if(cvWaitKey(33)=='d'){//capture the depth frame used for calculating depth accuracy
			cvCopy(depthframe,depth_image);
			break;
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////collect datas for calculate the depth accuracy from the captured depth image////////////////////////////////
	IplImage *color_image=cvCreateImage(cvSize(640,480),8,3);
	int Depth_x,Depth_y,IR_x,IR_y;//the image coordinate of one point respectively in depth image and IR image
	int u0=3,v0=3;//the shift between pixel of IR image an depth image
	vector <Point3D> D_point_vector;
	Point3D depth_point={0,0,0};
	CvPoint2D32f ptTest;
	for(int y=0;y<depthframe->height;y++){
		ushort *depth_ptr=(ushort*)(//深度数据为ushort类型
			depthframe->imageData+y*depthframe->widthStep
			);
		for(int x=0;x<depthframe->width;x++){
			Depth_x=x;
			Depth_y=y;
			IR_x=Depth_x+u0;
			IR_y=Depth_y+v0;
			ptTest.x=IR_x;//.......为ptTest赋值
			ptTest.y=IR_y;
			if(cvPointPolygonTest(pmatContour,ptTest,1)>0){//正在四边形内部，负在外部
				depth_point.x=Depth_x;
				depth_point.y=Depth_y;
				depth_point.depth=NuiDepthPixelToDepth(depth_ptr[x]);//提取深度值
				D_point_vector.push_back(depth_point);//collect the points used for calibration of depth camera
				//cvCopy(colorframe,color_image);
				//cvCircle(color_image,cvPoint(IR_x,IR_y),5,CV_RGB(255,0,0),2,8,0);
				//cvCopy(depthframe,depth_image);
				//cvCircle(depth_image,cvPoint(Depth_x,Depth_y),5,CV_RGB(255,0,0),2,8,0);
				//cvShowImage("calibrate depth camera",depth_image);
				//cvShowImage("IR frame",color_image);
				//cvWaitKey(1);//used for check whether the points used for calibrating depth camera are inside the chessboard
			}
		}
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////基于用于depth camera 的标定数据，计算出深度误差;深度误差即为pattern区域内部所有点的深度偏差值的绝对值 |Z_depth-Z_IR| 的平均值。//////////
	double average_error=0;
	double average_z=0;
	double average_d=0;
	double z,d;
	double board_side=0.075;
	CvMat *IR_Q_board_homogeneous=cvCreateMat(3,1,CV_32FC1);//齐次坐标 用于单应性变换
	CvMat *IR_Q_board=cvCreateMat(3,1,CV_32FC1);//chessboard坐标系下的3D坐标
	CvMat *IR_Q_camera=cvCreateMat(3,1,CV_32FC1);
	CvMat *IR_q=cvCreateMat(3,1,CV_32FC1);//图像点的齐次坐标
	int num=D_point_vector.size();
	for(int i=0;i<num;i++){
		depth_point=D_point_vector[i];
		Depth_x=depth_point.x;
		Depth_y=depth_point.y;
		d=depth_point.depth;//z coordinate of point Q in the coordinate system of depth camera
		IR_x=Depth_x+u0;
		IR_y=Depth_y+v0;
		//According to the kinect's feature, it's different from usual central perspective projection. So the horizontal coordinate(here, y is the horizontal coordinate) should be overturned.
		IR_y=480-IR_y;//将一个点对应到中心投影模型之后的坐标（Hd，R，T都是在中心投影模型下求得的）
		CV_MAT_ELEM(*IR_q,float,0,0)=IR_x;
		CV_MAT_ELEM(*IR_q,float,1,0)=IR_y;
		CV_MAT_ELEM(*IR_q,float,2,0)=1.0;
		cvMatMul(Hd,IR_q,IR_Q_board_homogeneous);
		//归一化
		CV_MAT_ELEM(*IR_Q_board_homogeneous,float,0,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,0,0)/CV_MAT_ELEM(*IR_Q_board_homogeneous,float,2,0);
		CV_MAT_ELEM(*IR_Q_board_homogeneous,float,1,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,1,0)/CV_MAT_ELEM(*IR_Q_board_homogeneous,float,2,0);
		//chessboard坐标系下的3D坐标
		CV_MAT_ELEM(*IR_Q_board,float,0,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,0,0);
		CV_MAT_ELEM(*IR_Q_board,float,1,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,1,0);
		CV_MAT_ELEM(*IR_Q_board,float,2,0)=0.0;
		//IR camera坐标系下的3D坐标 以board_side为单位
		cvMatMulAdd(rotation_matrix,IR_Q_board,translation_vector,IR_Q_camera);
		z=board_side*CV_MAT_ELEM(*IR_Q_camera,float,2,0);//z coordinate of point Q in the coordinate system of IR camera
		double error=z-1/((c1)*d+(c0));
		average_error+=abs(z-1/((c1)*d+(c0)))/num;
		average_z+=z/num;
		average_d+=d/num;
	}
	cout<<"average z coordinate: "<<average_z<<endl;
	cout<<"average d :"<<average_d<<endl;
	cvReleaseImage(&gray_image);
	cvReleaseImage(&colorframe);
	cvReleaseImage(&color_image);
	cvReleaseImage(&depthframe);
	cvReleaseImage(&depth_image);
	cvReleaseImage(&image);
	cvReleaseMat(&pmatContour);
	cvReleaseMat(&IR_Q_board_homogeneous);
	cvReleaseMat(&IR_Q_board);
	cvReleaseMat(&IR_Q_camera);
	cvReleaseMat(&IR_q);
	return average_error;//基于任意位置的pattern，计算得出的深度的平均误差；这里深度误差定义为pattern区域内部所有点的深度偏差值的绝对值 |Z_depth-Z_IR| 的平均值。
}
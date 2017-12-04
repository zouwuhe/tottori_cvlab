//Opencv 1.0 is used in this program.
//Calibrate the depth camera of Kinect (c0,c1)
//Firstly, extract and recognize the pattern from stream of IR images. Calculate the position of pattern and store related data.
//Secondly, open the stream of depth image. Use those points inside the area of pattern to optimize c0, c1.   X_D -> X_IR
//So after extracting those corners from IR image of pattern, it's neccesary to keep the chessboard and kinect hold steady until the depth image used for calibration is captured.
//Besides, had better set the distance from chessboard to Kinect as about 0.9m. 

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

const int board_dt=30;//wait 20 frames per chessboard view
int board_w;//the width of chessboard
int board_h;//the highth of chessboard
long angle;//the anlge of kinect1

struct Point3D{// the struct of 3D point of depth image
	int x;
	int y;
	ushort depth;
};//һ������kinect��depth camera �µ������Լ����ֵ

double calibrate_depth_camera1(vector<CvPoint2D32f> & pair_d_z,double *cc0,double *cc1);
double calibrate_depth_camera2(vector<CvPoint2D32f> & pair_d_z,double *cc0,double *cc1);
double evaluate_depth_accuracy(int board_dt,int board_w,int board_h,HANDLE hNextColorFrameEvent,HANDLE pColorStreamHandle,HANDLE hNextDepthFrameEvent,HANDLE pDepthStreamHandle,double c0,double c1);

int main(int argc,char* argv[]){
	cout<<"Input these parameters:"<<endl;
	cout<<"1. the width of chessboard"<<endl;
	cout<<"2. the highth of chessboard"<<endl;
	cout<<"4. the angle of kinect1"<<endl;
	cin>>board_w>>board_h>>angle;

	//read in the data from the IR camera of kinect //������ kinect1������kinect1��������Ȼ���ȡIRͼ��
	///initialize
	HRESULT hr=NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_COLOR|NUI_INITIALIZE_FLAG_USES_DEPTH|NUI_INITIALIZE_FLAG_USES_SKELETON);///NUI_INITIALIZE_FLAG_USES_DEPTH can be changed to NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
	if(hr!=S_OK){
		cout<<"NuiInitialize failed\n"<<endl;
		return hr;
	}
	//adjust the angle of the kinect1
	NuiCameraElevationSetAngle(angle);
	///create the color frame stream handle
	HANDLE hNextColorFrameEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	HANDLE pColorStreamHandle=NULL;
	///create the depth frame stream handle
	HANDLE hNextDepthFrameEvent=CreateEvent(NULL,TRUE,FALSE,NULL);
	HANDLE pDepthStreamHandle=NULL;
	//open the infrared frame stream
	hr=NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR_INFRARED,
		NUI_IMAGE_RESOLUTION_640x480,
		0,
		2,
		hNextColorFrameEvent,//�¼��������һ֡��ɫ����׼����ʱ�����¼����������
		&pColorStreamHandle);//�򿪵Ĳ�ɫ�������ľ���ĵ�ַ
	if(FAILED(hr)){
		cout<<"Can't open the infrared frame stream\n"<<endl;
		return hr;
	}
	///open the depth frame stream
	hr=NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH,
		NUI_IMAGE_RESOLUTION_640x480,
		NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE,//NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE
		2,
		hNextDepthFrameEvent,//�¼��������һ֡�������׼����ʱ�����¼����������
		&pDepthStreamHandle);//�򿪵�����������ľ���ĵ�ַ
	if(FAILED(hr)){
		cout<<"Can't open the depth frame stream\n"<<endl;
		return hr;
	}

	//prepare for the collection of chessboard
	int board_n=board_w*board_h;
	CvSize board_sz=cvSize(board_w,board_h);
	int corner_count;
	int frame=0;
	cvNamedWindow("IR frame");
	cvNamedWindow("IR pattern");
	cvNamedWindow("depth frame");
	cvNamedWindow("calibrate depth camera");
	IplImage *colorframe=cvCreateImage(cvSize(640,480),8,3);
	IplImage *color_image=cvCreateImage(cvSize(640,480),8,3);
	IplImage *image=cvCreateImage(cvSize(640,480),8,3);
	IplImage *gray_image=cvCreateImage(cvSize(640,480),8,1);
	IplImage *depthframe=cvCreateImage(cvSize(640,480),16,1);
	IplImage *depth_image=cvCreateImage(cvSize(640,480),16,1);

	vector <CvPoint2D32f> pair_d_z;
	int Depth_x,Depth_y,IR_x,IR_y;//the image coordinate of one point respectively in depth image and IR image
	int u0=3,v0=3;//the shift between pixel of IR image an depth image
	float board_side=0.075;
	bool IR_successes=0;

	//allocate storage for calibrate depth camera
	CvMat *Depth_intrinsic_matrix=cvCreateMat(1,2,CV_32FC1);
	CvMat *IR_intrinsic_matrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *IR_distortion_coeffs=cvCreateMat(4,1,CV_32FC1);
	CvMat *IR_image_points1=cvCreateMat(board_n,2,CV_32FC1);//used for calculate R,T of pattern
	CvMat *IR_object_points1=cvCreateMat(board_n,3,CV_32FC1);
	CvMat *IR_image_points2=cvCreateMat(board_n,3,CV_32FC1);//used for calculate homography
	CvMat *IR_object_points2=cvCreateMat(board_n,3,CV_32FC1);
	CvPoint2D32f *IR_corners=new CvPoint2D32f[board_n];
	CvMat *Hd=cvCreateMat(3,3,CV_32FC1);/////////////////used for calibrating depth camera
	CvMat *rotation_vector=cvCreateMat(3,1,CV_32FC1);
	CvMat *rotation_matrix=cvCreateMat(3,3,CV_32FC1);/////////////////used for calibrating depth camera
	CvMat *translation_vector=cvCreateMat(3,1,CV_32FC1);/////////////////used for calibrating depth camera
	//load these matrices back in
	IR_intrinsic_matrix=(CvMat*)cvLoad("calibrated_Intrinsic_IR2.xml");
	IR_distortion_coeffs=(CvMat*)cvLoad("calibrated_Distortion_IR2.xml");

	float pfVertex[8];
	int ContourNum=4;  //����ζ�������
	CvMat* pmatContour=cvCreateMat(1,ContourNum,CV_32FC2);    // pmatContour��һ��2ͨ����һά���飬����ͨ��  /////////////////used for calibrating depth camera
	CvPoint board_area[4];

	CvMat *IR_Q_board_homogeneous=cvCreateMat(3,1,CV_32FC1);//������� ���ڵ�Ӧ�Ա任
	CvMat *IR_Q_board=cvCreateMat(3,1,CV_32FC1);//chessboard����ϵ�µ�3D����
	CvMat *IR_Q_camera=cvCreateMat(3,1,CV_32FC1);
	CvMat *IR_q=cvCreateMat(3,1,CV_32FC1);//ͼ�����������
	float z_coordinate,d_value;
	char main_break;
	////////////////////////////////////////main loop for collecting data for the calibration of depth camera/////////////////////////////////////////////
	while(1){
		main_break=cvWaitKey(0);
		if(main_break==27) break;
		///////////////////////////////////////////////////////calculate the pattern from IR camera///////////////////////////////////////////////////////////
		while(!IR_successes){
			//read in the IR image in real time.
			if(WAIT_OBJECT_0==WaitForSingleObject(hNextColorFrameEvent,0)){
				///read the IR frame from color data stream
				const NUI_IMAGE_FRAME *pImageFrame=NULL; //buffer for color data
				hr=NuiImageStreamGetNextFrame(
					pColorStreamHandle,
					0,//��������ȴ���ͼ��֡��ֱ�ӷ��ص�ʱ��
					&pImageFrame);//��ָ����������ȡ��һ֡��Ϣ����Ϣ��ַ�����NUI_IMAGE_FRAME�Ľṹ��ָ�룬�ṹ���е�INuiFrameTexture����֡����
				if(FAILED(hr)){
					cout<<"Get infrared frame failed\n"<<endl;
					continue;
				}
				INuiFrameTexture *pTexture=pImageFrame->pFrameTexture; //��NUI_IMAGE_FRAME�ṹ���У���ȡ����Ҫ��֡����
				NUI_LOCKED_RECT lockedrect;//����һ���ṹ�壬���ڱ��������������Ϣ
				pTexture->LockRect(0,&lockedrect,NULL,0);
				if(lockedrect.Pitch!=0){//pitch���ػ�������һ�е��ֽ���
					BYTE *pBuffer=(BYTE*)lockedrect.pBits;//�������׵�ַ
					for(int y=0;y<480;y++){
						uchar* color_ptr=(uchar*)(//��ɫ����Ϊuchar����
							colorframe->imageData+y*colorframe->widthStep
							);					
						for(int x=0;x<640;x++){
							BYTE intensity = reinterpret_cast<USHORT*>(lockedrect.pBits)[y*640+x] >> 8;//���infrared�������Ķ�ȡ��ʽ���������ɫ�������Ķ�ȡ��ʽ��
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
			if(frame++%board_dt==0){//ÿ���20֡����һ�� �ɼ�ͼ���ϵ����̽ǵ�
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
				cvWaitKey(33);//��ʾͼ������Ҫʱ��ģ����Ա����һ����ʱ�������޷�ͬ������ͼ��
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
						CV_MAT_ELEM(*IR_object_points2,float,i,1)=(float)(i%board_w);//ע�⣬�˴���baord_w��Ϊy
						CV_MAT_ELEM(*IR_object_points2,float,i,2)=1.0f;
					}
					cvFindHomography(IR_image_points2,IR_object_points2,Hd);//���㵥Ӧ�Ծ��󣬴�ͼ����Ӧ��pattern�ϵ������
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
				cvDestroyWindow("IR frame");
				cvDestroyWindow("IR pattern");
				cvReleaseImage(&gray_image);
				cvReleaseImage(&colorframe);
				cvReleaseImage(&depthframe);
				cvReleaseImage(&depth_image);
				cvReleaseImage(&image);
				NuiShutdown();
				return 0;
			}
		}
		IR_successes=false;
		////////////////////////////////////////
		///////////////////�����ı������ڱ궨depth camera������ѡȡ�궨���ڲ��ĵ�
		//float value_fVertex=0;
		//float* pfVertex=&value_fVertex; //����ζ���λ��
		//.....ΪpfVertex��ֵ
		pfVertex[0]=IR_corners[0].x;
		pfVertex[1]=IR_corners[0].y;
		pfVertex[2]=IR_corners[board_w-1].x;
		pfVertex[3]=IR_corners[board_w-1].y;
		pfVertex[4]=IR_corners[board_n-1].x;
		pfVertex[5]=IR_corners[board_n-1].y;
		pfVertex[6]=IR_corners[board_n-board_w].x;
		pfVertex[7]=IR_corners[board_n-board_w].y;
		for (int i=0;i<ContourNum;i++)
		{
			CvScalar vertex = cvScalar(pfVertex[2*i], pfVertex[2*i+1]);       // һ����ά����
			cvSet1D(pmatContour,i,vertex);                // ʹ��CvScalar��Ϊ����Ԫ��
		}
		//pmatContour ��Ϊ�ı������������ں��������Ƿ��ڱ궨��������
		//////////////////////////////////////// �����ı�������
		/*CvPoint value1_board_area=cvPoint(0,0);
		CvPoint *board_area=&value1_board_area;*/
		board_area[0]=cvPoint((int)IR_corners[0].x,(int)IR_corners[0].y);
		board_area[1]=cvPoint((int)IR_corners[board_w-1].x,(int)IR_corners[board_w-1].y);
		board_area[2]=cvPoint((int)IR_corners[board_n-1].x,(int)IR_corners[board_n-1].y);
		board_area[3]=cvPoint((int)IR_corners[board_n-board_w].x,(int)IR_corners[board_n-board_w].y);
		cvFillConvexPoly(colorframe,board_area,4,CV_RGB(255,255,255));

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
						ushort* depth_ptr=(ushort*)(//�������Ϊushort����
							depthframe->imageData+y*depthframe->widthStep
							);
						uchar *pBufferRun=(uchar*)(lockedrect.pBits)+y*lockedrect.Pitch;//pitch���ص��ǻ�����һ�е��ֽ���
						ushort*pBuffer=(ushort*)pBufferRun;
						for(int x=0;x<640;x++){
							depth_ptr[x]=pBuffer[x];
							//////////ptr[j] = 255 - (uchar)(256 * pBuffer[j]/0x0fff);  //Ҳ����ֱ�ӽ����ݹ�һ���� ���Ϊchar�� 8bit���ͼ
						}
					}
					/*//��ʾx160y120λ���ϵ�������Ϣ
					ushort* depth_ptr=(ushort*)(//�������Ϊushort����
					depthframe->imageData+120*depthframe->widthStep
					);//��ʱ���ͼ��Ϊushort���ݸ�ʽ
					printf("x:160 y:120���괦�����ֵ��d=%d\n",depth_ptr[160]);*/
					cvShowImage("depth frame",depthframe);
					cvWaitKey(33);
				}
				else{
					cout<<"Bufferlength of received texture from depth buffer is bogus\r\n"<<endl;
				}
				///release the data of this frame and be ready for next frame in the color stream
				NuiImageStreamReleaseFrame(pDepthStreamHandle,pImageFrame);
				/*if(lockedrect.Pitch!=0){
				ushort*pBuffer=(ushort*)((uchar*)(lockedrect.pBits)+119*lockedrect.Pitch);//��ȡ������ݻ��������е��������ֵ������ushort����ʽ������pBuffer��16bit������λΪplayer������
				//��ʾx160y120λ���ϵ�������Ϣ
				printf("x:160 y:120���괦�����ֵ��d=%d\n",pBuffer[160]);//�������Ϊushort�������ͣ�������ݷ�Χ0��65535,16bit ֱ����%d ��ʮ������ʽ��� ���ڵ�3λΪ0�����Ϊ8�ı���
				}*/
			}
			if(cvWaitKey(33)=='d'){//capture the depth frame used for calibrating depth camera
				cvCopy(depthframe,depth_image);
				break;
			}
		}

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////collect datas for calibration of the depth camera////////////////////////////////////
		vector <Point3D> D_point_vector;
		Point3D depth_point={0,0,0};
		CvPoint2D32f ptTest;
		for(int y=0;y<depthframe->height;y++){
			ushort *depth_ptr=(ushort*)(//�������Ϊushort����
				depthframe->imageData+y*depthframe->widthStep
				);
			for(int x=0;x<depthframe->width;x++){
				Depth_x=x;
				Depth_y=y;
				IR_x=Depth_x+u0;
				IR_y=Depth_y+v0;
				ptTest.x=IR_x;//.......ΪptTest��ֵ
				ptTest.y=IR_y;
				if(cvPointPolygonTest(pmatContour,ptTest,1)>0){//�����ı����ڲ��������ⲿ
					depth_point.x=Depth_x;
					depth_point.y=Depth_y;
					depth_point.depth=NuiDepthPixelToDepth(depth_ptr[x]);//��ȡ���ֵ
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

		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/////////////////////////////////////////collect points (d,z) for calibration of the depth camera///////////////////////////////////////
		int num1=D_point_vector.size();
		//////////////////collect data of points
		for(int i=0;i<num1;i++){
			depth_point=D_point_vector[i];
			Depth_x=depth_point.x;
			Depth_y=depth_point.y;
			d_value=depth_point.depth;//z coordinate of point Q in the coordinate system of depth camera
			IR_x=Depth_x+u0;
			IR_y=Depth_y+v0;
			//According to the kinect's feature, it's different from usual central perspective projection. So the horizontal coordinate(here, y is the horizontal coordinate) should be overturned.
			IR_y=480-IR_y;//��һ�����Ӧ������ͶӰģ��֮������꣨Hd��R��T����������ͶӰģ������õģ�
			CV_MAT_ELEM(*IR_q,float,0,0)=IR_x;
			CV_MAT_ELEM(*IR_q,float,1,0)=IR_y;
			CV_MAT_ELEM(*IR_q,float,2,0)=1.0;
			cvMatMul(Hd,IR_q,IR_Q_board_homogeneous);
			//��һ��
			CV_MAT_ELEM(*IR_Q_board_homogeneous,float,0,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,0,0)/CV_MAT_ELEM(*IR_Q_board_homogeneous,float,2,0);
			CV_MAT_ELEM(*IR_Q_board_homogeneous,float,1,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,1,0)/CV_MAT_ELEM(*IR_Q_board_homogeneous,float,2,0);
			//chessboard����ϵ�µ�3D����
			CV_MAT_ELEM(*IR_Q_board,float,0,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,0,0);
			CV_MAT_ELEM(*IR_Q_board,float,1,0)=CV_MAT_ELEM(*IR_Q_board_homogeneous,float,1,0);
			CV_MAT_ELEM(*IR_Q_board,float,2,0)=0.0;
			//IR camera����ϵ�µ�3D���� ��board_sideΪ��λ
			cvMatMulAdd(rotation_matrix,IR_Q_board,translation_vector,IR_Q_camera);
			z_coordinate=board_side*CV_MAT_ELEM(*IR_Q_camera,float,2,0);//z coordinate of point Q in the coordinate system of IR camera
			pair_d_z.push_back(cvPoint2D32f(d_value,z_coordinate));
		}
	}

	//////////////////calibrate the depth camera (c0,c1) according to the datas collected (Hd,rotation_matrix,translation_vector,D_point_vector)////////////////
	/////////////////////////// evaluate the calibration accuracy of depth camera based on the image used for calibrating depth camera /////////////////////////
	double c0,c1,ca0,ca1,cb0,cb1;
	double average_error1,average_error2;
	double average_error3;
	average_error1=calibrate_depth_camera1(pair_d_z,&ca0,&ca1);
	cout<<"calibrated result for depth camera(c0,c1) by fitting the line: "<<ca0<<' '<<ca1<<endl;
	cout<<"average error: "<<average_error1<<endl;
	cout<<endl;
	average_error2=calibrate_depth_camera2(pair_d_z,&cb0,&cb1);
	cout<<"calibrated result for depth camera(c0,c1) by solve linear overdetermined equation: "<<cb0<<' '<<cb1<<endl;
	cout<<"average error: "<<average_error2<<endl;
	c0=ca0;
	c1=ca1;
	if(average_error2<average_error1){
		c0=cb0;
		c1=cb1;
	}
	CV_MAT_ELEM(*Depth_intrinsic_matrix,float,0,0)=c0;
	CV_MAT_ELEM(*Depth_intrinsic_matrix,float,0,1)=c1;
	cvSave("calibrated_Intrinsic_Depth2.xml",Depth_intrinsic_matrix);
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////// evaluate the calibration accuracy of depth camera based on images of different ranges //////////////////////////////////
	//////////////////////////////////////////////////�ı�pattern��kinect�����λ�ã����Բ�ͬλ�õ�������//////////////////////////////////////////////////////
	//////////////////////////////////////////////////pattern ͨ��IR camera ��ò�ʶ��depthͨ�� depth camera ����//////////////////////////////////////////////
	while(1){
		average_error3=evaluate_depth_accuracy(board_dt,board_w,board_h,hNextColorFrameEvent,pColorStreamHandle,hNextDepthFrameEvent,pDepthStreamHandle,c0,c1);
		cout<<"depth accuracy: "<<average_error3<<endl;
		cout<<endl;
		char c=cvWaitKey(0);
		if(c==27)
			break;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	cvDestroyWindow("IR frame");
	cvDestroyWindow("IR pattern");
	cvDestroyWindow("depth frame");
	cvDestroyWindow("calibrate depth camera");
	////////////////////////////////////////
	delete [] IR_corners;
	cvReleaseMat(&Depth_intrinsic_matrix);
	cvReleaseMat(&IR_object_points1);
	cvReleaseMat(&IR_image_points1);
	cvReleaseMat(&IR_object_points2);
	cvReleaseMat(&IR_image_points2);
	cvReleaseMat(&IR_intrinsic_matrix);
	cvReleaseMat(&IR_distortion_coeffs);
	cvReleaseMat(&rotation_vector);
	cvReleaseMat(&pmatContour);
	cvReleaseImage(&gray_image);
	cvReleaseImage(&colorframe);
	cvReleaseImage(&color_image);
	cvReleaseImage(&depthframe);
	cvReleaseImage(&depth_image);
	cvReleaseImage(&image);
	///shut down the nui
	NuiShutdown();
	return 0;
}
//ע�� ��¡���� cvCloneImage() ÿ��ʹ�û����·���ռ䣬��˸ú�����ѭ����Ҫ����ʹ��
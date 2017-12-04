//输入用于标定depth camera的深度点序列 (d,z)；
//标定depth camera：cc0,cc1。并基于用于标定的数据，计算出标定误差。
//标定误差即为pattern区域内部所有点的深度偏差值的绝对值 |Z_depth-Z_IR| 的平均值。标定误差作为函数返回值
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////直线拟合法，标定depth camera(c0,c1)////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <cxcore.h>
#include <cvcam.h>
#include <iostream>
#include <vector>
using namespace std;

struct Point3D{// the struct of 3D point of depth image
	int x;
	int y;
	ushort depth;
};//一个点在kinect的depth camera 下的坐标以及深度值

double calibrate_depth_camera1(vector<CvPoint2D32f> & pair_d_z,double *cc0,double *cc1){
	float line[4];
	CvPoint2D32f* points = (CvPoint2D32f*)malloc(pair_d_z.size()* sizeof(points[0])); //存放产生的点，数目为 point_vector.size()
	for(int i=0;i<pair_d_z.size();i++){
		points[i].x=pair_d_z[i].y;
		points[i].y=pair_d_z[i].y*pair_d_z[i].x;
	}
	CvMat pointMat = cvMat(1,pair_d_z.size(),CV_32FC2,points); //点集, 存储count个随机点points  
	cvFitLine(&pointMat,CV_DIST_L1,0,0.01,0.01,line);
	//////////////////calculate c0 and c1 from the result of line fitting
	double A,B,C;
	A=-line[1];
	B=line[0];
	C=line[1]*line[2]-line[0]*line[3];
	*cc0=-A/C;
	*cc1=-B/C;
	/////////////////基于用于标定的数据，计算出标定误差;标定误差即为pattern区域内部所有点的深度偏差值的绝对值 |Z_depth-Z_IR| 的平均值。/////////////////
	int num=pair_d_z.size();
	double average_error=0;
	double average_z=0;
	double average_d=0;
	double z,d;
	for(int i=0;i<num;i++){
		d=pair_d_z[i].x;
		z=pair_d_z[i].y;
		average_error+=abs(z-1/((*cc1)*d+(*cc0)))/num;
		average_z+=z/num;
		average_d+=d/num;
	}
	cout<<"average z coordinate: "<<average_z<<endl;
	cout<<"average d :"<<average_d<<endl;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	free(points);//注意malloc 与new 的分配与释放方法不一样
	return average_error;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////求解线性方程组，标定depth camera(c0,c1)////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////

double calibrate_depth_camera2(vector<CvPoint2D32f> & pair_d_z,double *cc0,double *cc1){
	int num=pair_d_z.size();
	CvMat *points=cvCreateMat(num,2,CV_32FC1);
	double z,d;
	float line[4];
	//////////////////collect data of points, and prepare for fit line
	for(int i=0;i<num;i++){
		d=pair_d_z[i].x;
		z=pair_d_z[i].y;
		CV_MAT_ELEM(*points,float,i,0)=z;
		CV_MAT_ELEM(*points,float,i,1)=z*d;
	}
	//////////////////solve the system of linear equations
	CvMat *points_transpose=cvCreateMat(2,num,CV_32FC1);
	CvMat *matrix_u=cvCreateMat(2,2,CV_32FC1);
	CvMat *matrix_u_invert=cvCreateMat(2,2,CV_32FC1);
	CvMat *matrix_X=cvCreateMat(2,1,CV_32FC1);//the matrix of c0 and c1
	CvMat *matrix_I=cvCreateMat(num,1,CV_32FC1);
	CvMat *matrix_Y=cvCreateMat(2,1,CV_32FC1);
	cvSet(matrix_I,cvScalarAll(1.0));
	cvTranspose(points,points_transpose);
	cvMatMul(points_transpose,points,matrix_u);
	cvInvert(matrix_u,matrix_u_invert);
	cvMatMul(points_transpose,matrix_I,matrix_Y);
	cvMatMul(matrix_u_invert,matrix_Y,matrix_X);
	//cvSolve(points,matrix_X,matrix_I,CV_LU);//CV_SVD
	*cc0=CV_MAT_ELEM(*matrix_X,float,0,0);
	*cc1=CV_MAT_ELEM(*matrix_X,float,1,0);
	/////////////////基于用于标定的数据，计算出标定误差;标定误差即为pattern区域内部所有点的深度偏差值的绝对值 |Z_depth-Z_IR| 的平均值。/////////////////
	double average_error=0;
	double average_z=0;
	double average_d=0;
	for(int i=0;i<num;i++){
		d=pair_d_z[i].x;
		z=pair_d_z[i].y;
		average_error+=abs(z-1/((*cc1)*d+(*cc0)))/num;
		average_z+=z/num;
		average_d+=d/num;
	}
	cout<<"average z coordinate: "<<average_z<<endl;
	cout<<"average d :"<<average_d<<endl;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	cvReleaseMat(&points);
	cvReleaseMat(&matrix_X);
	cvReleaseMat(&matrix_I);
	return average_error;
}
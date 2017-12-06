//observed quantity: fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz; 
//parameter:Tx,Ty,Tz,a,b,c,d corresponding to x[0],x[1]... ...x[6];
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cv.h>
#include "prepare_function.h"
using namespace std;
void store_calibration_result(string filename,double *x,int k,double adjustment1,double adjustment2,double time);

void calibration(string filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int m){
	double t = (double)cvGetTickCount();
	//define the parameter for the iterative loop
	int k=0,k_max=900000,v=6;
	double e1=0.000000000000000001,e2=0.0000000000000000000000001,tao=0.01;
	double judgement1;//收敛程度观察量
	double judgement2;//收敛程度观察量
	double x_new[7]={0};
	double h[7]={0};
	double A[7*7]={0};
	double AuI[7*7]={0};
	double AuI_invert[7*7]={0};
	double g[7]={0};
	double h_t[7]={0};
	double hg[7]={0};
	double hhg[1]={0};
	double *f_x=(double *)malloc(m*sizeof(double));//f_x present a function of x
	double *f_x_new=(double *)malloc(m*sizeof(double));
	double *Jx=(double *)malloc(m*7*sizeof(double));
	double *Jx_t=(double *)malloc(m*7*sizeof(double));//Jx的转置
	double I[7*7]={0}; //单位矩阵
	I[0]=1;I[8]=1;I[16]=1;I[24]=1;I[32]=1;I[40]=1;I[48]=1;
	///r1,r2,r3,Px,Py,Pz,Qx,Qy,Qz 为已知量  参数x[0]-x[7] 已经赋值  准备矩阵 fx Jx
	CvMat R_fx=cvMat(m,1,CV_64FC1,f_x);
	CvMat R_fx_new=cvMat(m,1,CV_64FC1,f_x_new);
	CvMat R_Jx=cvMat(m,7,CV_64FC1,Jx);
	CvMat R_Jx_t=cvMat(7,m,CV_64FC1,Jx_t);//Jx的转置
	CvMat R_A=cvMat(7,7,CV_64FC1,A);
	CvMat R_g=cvMat(7,1,CV_64FC1,g);
	CvMat R_AuI=cvMat(7,7,CV_64FC1,AuI);
	CvMat R_I=cvMat(7,7,CV_64FC1,I);
	CvMat R_AuI_invert=cvMat(7,7,CV_64FC1,AuI_invert);
	CvMat R_h=cvMat(7,1,CV_64FC1,h);
	CvMat R_h_t=cvMat(1,7,CV_64FC1,h_t);
	CvMat R_x=cvMat(7,1,CV_64FC1,x);
	CvMat R_x_new=cvMat(7,1,CV_64FC1,x_new);
	CvMat R_hg=cvMat(7,1,CV_64FC1,hg);
	CvMat R_hhg=cvMat(1,1,CV_64FC1,hhg);
	//calculate the matrix fx
	for(int i=0;i<m;i++){
		f_x[i]=f_objective(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
	}
	//calculate the matrix Jx
	for(int j=0;j<m*7;j++){
		int i=j/7;
		switch(j-i*7){
		case 0:{
			Jx[j]=pd_f_Tx(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 1:{
			Jx[j]=pd_f_Ty(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 2:{
			Jx[j]=pd_f_Tz(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 3:{
			Jx[j]=pd_f_a(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 4:{
			Jx[j]=pd_f_b(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 5:{
			Jx[j]=pd_f_c(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 6:{
			Jx[j]=pd_f_d(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		}
	}
	///calculate matrix A and g
	cvTranspose(&R_Jx,&R_Jx_t);
	cvMatMul(&R_Jx_t,&R_Jx,&R_A);
	cvMatMul(&R_Jx_t,&R_fx,&R_g);
	int found=(cvNorm(&R_g,NULL,CV_L2)<=e1);
	double u=tao*max(A[0],A[8],A[16],A[24],A[32],A[40],A[48]);
	///main loop for iterative process
	while(!found&&k<k_max){
		k++;
		cvScaleAdd(&R_I,cvRealScalar(u),&R_A,&R_AuI);
		cvInvert(&R_AuI,&R_AuI_invert);
		cvGEMM(&R_AuI_invert,&R_g,-1,NULL,NULL,&R_h);
		if(cvNorm(&R_h,NULL,CV_L2)<=e2*(cvNorm(&R_x,NULL,CV_L2)+e2))
			found=true;
		else{
			cvAdd(&R_x,&R_h,&R_x_new);
			for(int i=0;i<m;i++){
				f_x_new[i]=f_objective(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x_new);
			}
			double Fx=pow(cvNorm(&R_fx,NULL,CV_L2),2)/2.0;
			double Fx_new=pow(cvNorm(&R_fx_new,NULL,CV_L2),2)/2.0;
			cvAddWeighted(&R_h,u,&R_g,-1,0,&R_hg);
			cvTranspose(&R_h,&R_h_t);
			cvMatMul(&R_h_t,&R_hg,&R_hhg);
			double e=(Fx-Fx_new)/(hhg[0]/2.0);
			if(e>0){
				cvCopy(&R_x_new,&R_x);
				//calculate the matrix fx
				for(int i=0;i<m;i++){
					f_x[i]=f_objective(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);//
				}
				//calculate the matrix Jx
				for(int j=0;j<m*7;j++){
					int i=j/7;
					switch(j-i*7){
					case 0:{
						Jx[j]=pd_f_Tx(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 1:{
						Jx[j]=pd_f_Ty(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 2:{
						Jx[j]=pd_f_Tz(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 3:{
						Jx[j]=pd_f_a(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 4:{
						Jx[j]=pd_f_b(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 5:{
						Jx[j]=pd_f_c(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 6:{
						Jx[j]=pd_f_d(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					}
				}
				///calculate matrix A and g
				cvTranspose(&R_Jx,&R_Jx_t);
				cvMatMul(&R_Jx_t,&R_Jx,&R_A);
				cvMatMul(&R_Jx_t,&R_fx,&R_g);
				judgement1=cvNorm(&R_g,NULL,CV_L2);//test
				judgement2=cvNorm(&R_h,NULL,CV_L2);//test
				found=(cvNorm(&R_g,NULL,CV_L2)<=e1);
				u=u*max_x_y(1.0/3,1-pow((2*e-1),3));
				v=2;
			}
			else{
				u=u*v;
				v=2*v;
			}
		}
	}
	t=((double)cvGetTickCount() - t)/(cvGetTickFrequency()*1000);
	store_calibration_result(filename,x,k,judgement1,judgement2,t);
	free(f_x);
	free(f_x_new);
	free(Jx);
	free(Jx_t);
}

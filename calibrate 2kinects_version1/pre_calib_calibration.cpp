//observed quantity:qa,qb,n1,n2,n3,Tpx,Tpy,Tpz;
//parameter:a,b,c,d corresponding to x[0],x[1] x[2] x[3];
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cv.h>
#include "pre_calib_prepare_function.h"

using namespace std;
void pre_calib_store_calibration_result(string filename,double *x,int k,double adjustment1,double adjustment2,double time);

void pre_calibration(string filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int m){
	double t = (double)cvGetTickCount();
	//define the parameter for the iterative loop
	int k=0,k_max=900000,v=6;
	double e1=0.000000000000000001,e2=0.0000000000000000000000001,tao=0.01;
	double judgement1;//收敛程度观察量
	double judgement2;//收敛程度观察量
	double x_new[4]={0};
	double h[4]={0};
	double A[4*4]={0};
	double AuI[4*4]={0};
	double AuI_invert[4*4]={0};
	double g[4]={0};
	double h_t[4]={0};
	double hg[4]={0};
	double hhg[1]={0};
	double *f_x=(double *)malloc(m*sizeof(double));//f_x present a function of x
	double *f_x_new=(double *)malloc(m*sizeof(double));
	double *Jx=(double *)malloc(m*4*sizeof(double));
	double *Jx_t=(double *)malloc(m*4*sizeof(double));//Jx的转置
	double I[4*4]={0}; //单位矩阵
	I[0]=1;I[5]=1;I[10]=1;I[15]=1;
	////////////////////////////////////////
	CvMat R_fx=cvMat(m,1,CV_64FC1,f_x);
	CvMat R_fx_new=cvMat(m,1,CV_64FC1,f_x_new);
	CvMat R_Jx=cvMat(m,4,CV_64FC1,Jx);
	CvMat R_Jx_t=cvMat(4,m,CV_64FC1,Jx_t);//Jx的转置
	CvMat R_A=cvMat(4,4,CV_64FC1,A);
	CvMat R_g=cvMat(4,1,CV_64FC1,g);
	CvMat R_AuI=cvMat(4,4,CV_64FC1,AuI);
	CvMat R_I=cvMat(4,4,CV_64FC1,I);
	CvMat R_AuI_invert=cvMat(4,4,CV_64FC1,AuI_invert);
	CvMat R_h=cvMat(4,1,CV_64FC1,h);
	CvMat R_h_t=cvMat(1,4,CV_64FC1,h_t);
	CvMat R_x=cvMat(4,1,CV_64FC1,x);
	CvMat R_x_new=cvMat(4,1,CV_64FC1,x_new);
	CvMat R_hg=cvMat(4,1,CV_64FC1,hg);
	CvMat R_hhg=cvMat(1,1,CV_64FC1,hhg);
	//calculate the matrix fx
	for(int i=0;i<m;i++){
		f_x[i]=pre_f_objective(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
	}
	//calculate the matrix Jx
	for(int j=0;j<m*4;j++){
		int i=j/4;
		switch(j-i*4){
		case 0:{
			Jx[j]=pre_pd_f_a(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 1:{
			Jx[j]=pre_pd_f_b(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 2:{
			Jx[j]=pre_pd_f_c(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		case 3:{
			Jx[j]=pre_pd_f_d(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
			   }
			   break;
		}
	}
	///calculate matrix A and g
	cvTranspose(&R_Jx,&R_Jx_t);
	cvMatMul(&R_Jx_t,&R_Jx,&R_A);
	cvMatMul(&R_Jx_t,&R_fx,&R_g);
	int found=(cvNorm(&R_g,NULL,CV_L2)<=e1);
	double u=tao*pre_max(A[0],A[5],A[10],A[15]);
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
				f_x_new[i]=pre_f_objective(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x_new);
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
					f_x[i]=pre_f_objective(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);//
				}
				//calculate the matrix Jx
				for(int j=0;j<m*4;j++){
					int i=j/4;
					switch(j-i*4){
					case 0:{
						Jx[j]=pre_pd_f_a(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 1:{
						Jx[j]=pre_pd_f_b(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 2:{
						Jx[j]=pre_pd_f_c(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
						   }
						   break;
					case 3:{
						Jx[j]=pre_pd_f_d(fx,fy,cx,cy,qa[i],qb[i],n1[i],n2[i],n3[i],Tpx[i],Tpy[i],Tpz[i],x);
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
				u=u*pre_max_x_y(1.0/3,1-pow((2*e-1),3));
				v=2;
			}
			else{
				u=u*v;
				v=2*v;
			}
		}
	}
	t=((double)cvGetTickCount() - t)/(cvGetTickFrequency()*1000);
	pre_calib_store_calibration_result(filename,x,k,judgement1,judgement2,t);
	free(f_x);
	free(f_x_new);
	free(Jx);
	free(Jx_t);
}
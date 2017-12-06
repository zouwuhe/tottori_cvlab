#include <fstream>
#include <iostream>
#include <string>
#include <highgui.h>
using namespace std;
void calibration(string filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int m);
void pre_calibration(string filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int m);
void epipole_accuracy(string filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int m);

void calibrate_2kinects_version1(string filename){
	ifstream infile(filename);
	if(!infile){
		cout<<endl<<"Failed to open file"<<filename;
		cvWaitKey(0);
	}
	int test_num;
	infile>>test_num;
	//constant parameter
	double fx,fy,cx,cy;
	double factor=30;//为了便于快速收敛，加入一个尺度因子 这里将尺度因子定为30
	//观测量
	int *qa=(int *)malloc(test_num*sizeof(int));
	int *qb=(int *)malloc(test_num*sizeof(int));
	double *n1=(double *)malloc(test_num*sizeof(double));
	double *n2=(double *)malloc(test_num*sizeof(double));
	double *n3=(double *)malloc(test_num*sizeof(double));
	double *Tpx=(double *)malloc(test_num*sizeof(double));
	double *Tpy=(double *)malloc(test_num*sizeof(double));
	double *Tpz=(double *)malloc(test_num*sizeof(double));
	double *direct_x=(double *)malloc(7*sizeof(double));
	double *pre_x=(double *)malloc(4*sizeof(double));
	double *sec_x=(double *)malloc(7*sizeof(double));
	infile>>fx;
	infile>>fy;
	infile>>cx;
	infile>>cy;
	for(int i=0;i<test_num;i++){
		infile>>*(qa+i);
		infile>>*(qb+i);
		infile>>*(n1+i);
		infile>>*(n2+i);
		infile>>*(n3+i);
		//对n1，n2，n3归一化
		double s=sqrt(pow(*(n1+i),2.0)+pow(*(n2+i),2.0)+pow(*(n3+i),2.0));
		*(n1+i)=*(n1+i)/s;
		*(n2+i)=*(n2+i)/s;
		*(n3+i)=*(n3+i)/s;
		//为了便于快速收敛，对归一化以后的激光方向 加入一个尺度因子 factor
		*(n1+i)=*(n1+i)*factor;
		*(n2+i)=*(n2+i)*factor;
		*(n3+i)=*(n3+i)*factor;
		////////////////////////////////////////////////////////////////
		infile>>*(Tpx+i);
		infile>>*(Tpy+i);
		infile>>*(Tpz+i);
	}
	for(int i=0;i<7;i++){
		infile>>*(direct_x+i);
	}
	infile.close();
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//directly calibrate the 2 kinects
	ofstream outfile(filename,ios::out|ios::app);
	outfile<<endl;
	outfile<<"direct calibration:"<<endl;
	outfile.close();
	calibration(filename,fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz,direct_x,test_num);
	//calculate the epipole accuracy of direct calibration result
	epipole_accuracy(filename,fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz,direct_x,test_num);
	//previous calibrate the 2 kinects
	outfile.open(filename,ios::out|ios::app);
	outfile<<endl;
	outfile<<"previous calibration:"<<endl;
	outfile.close();
	*(pre_x+0)=1;*(pre_x+1)=0;*(pre_x+2)=0;*(pre_x+3)=0;//给预标定的pre_x赋初值
	pre_calibration(filename,fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz,pre_x,test_num);
	//secondary calibrate the 2 kinects
	outfile.open(filename,ios::out|ios::app);
	outfile<<endl;
	outfile<<"secondary calibration:"<<endl;
	outfile.close();
	for(int i=0;i<7;i++){
		*(sec_x+i)=0;
	}
	*(sec_x+3)=*(pre_x+0);
	*(sec_x+4)=*(pre_x+1);
	*(sec_x+5)=*(pre_x+2);
	*(sec_x+6)=*(pre_x+3);//给二次标定的sec_x赋初值
	calibration(filename,fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz,sec_x,test_num);
	//calculate the epipole accuracy of secondary calibration result
	epipole_accuracy(filename,fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz,sec_x,test_num);
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	free(qa);
	free(qb);
	free(n1);
	free(n2);
	free(n3);
	free(Tpx);
	free(Tpy);
	free(Tpz);
	free(direct_x);
	free(pre_x);
	free(sec_x);
}
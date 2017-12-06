//基于测量数据，利用四元数法，标定R,T.
#include <fstream>
#include <iostream>
using namespace std;
void calibration(char *filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int m);
int main(){
	char *filename="simulation2_3.txt";
	ifstream infile(filename);
	if(!infile){
		cout<<endl<<"Failed to open file"<<filename;
		return 1;
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
	double *x=(double *)malloc(7*sizeof(double));
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
		infile>>*(x+i);
	}
	infile.close();
	calibration(filename,fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz,x,test_num);//calibrate the system parameter
	free(qa);
	free(qb);
	free(n1);
	free(n2);
	free(n3);
	free(Tpx);
	free(Tpy);
	free(Tpz);
	free(x);
	return 0;
}
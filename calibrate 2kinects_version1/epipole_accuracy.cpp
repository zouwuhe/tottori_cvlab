#include <fstream>
#include <iostream>
using namespace std;
double d_epipole(double cx,double cy,double fx,double fy, double h,double T1,double T2,double T3,double a,double b,double c,double d,double Tpx,double Tpy,double Tpz,double n1,double n2,double n3,double a0,double b0);
void epipole_accuracy(string filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int m){
	ofstream outfile(filename,ios::out|ios::app);
	outfile<<"test number:"<<m<<endl;// output to file
	outfile<<"Kinect2 constant parameter fx,fy,cx,cy:"<<fx<<' '<<fy<<' '<<cx<<' '<<cy<<endl;//output to file// the fx,fy,cx,cy is belong to kinect2
	outfile<<"Observed quantity qa,qb,n1,n2,n3,Tpx,Tpy,Tpz,epipole accuracy:"<<endl;
	double d_average=0;
	for(int i=0;i<m;i++){
		double d=d_epipole(cx,cy,fx,fy,480.0,*(x+0),*(x+1),*(x+2),*(x+3),*(x+4),*(x+5),*(x+6),*(Tpx+i),*(Tpy+i),*(Tpz+i),*(n1+i),*(n2+i),*(n3+i),*(qa+i),*(qb+i));
		d_average+=d/m;
		outfile<<*(qa+i)<<' '<<*(qb+i)<<' '<<*(n1+i)<<' '<<*(n2+i)
						<<' '<<*(n3+i)<<' '<<*(Tpx+i)<<' '<<*(Tpy+i)<<' '<<*(Tpz+i)<<' '<<d<<endl;
	}
	outfile<<*(x+0)<<' '<<*(x+1)<<' '<<*(x+2)<<' '<<*(x+3)<<' '<<*(x+4)<<' '<<*(x+5)<<' '<<*(x+6)<<endl;
	outfile<<"accuracy analysis epipole:"<<d_average<<endl;
	outfile.close();
}
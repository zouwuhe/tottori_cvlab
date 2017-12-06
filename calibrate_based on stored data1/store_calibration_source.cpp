#include <fstream>
using namespace std;
void store_calibration_source(char *filename,double fx,double fy,double cx,double cy,int *qa,int *qb,double *n1,double *n2,double *n3,double *Tpx,double *Tpy,double *Tpz,double *x,int test_num){
	ofstream outfile(filename);
	outfile<<"test number:"<<test_num<<endl;
	outfile<<"constant parameter fx,fy,cx,cy:"<<fx<<' '<<fy<<' '<<cx<<' '<<cy<<endl;
	outfile<<"simulation observed quantity qa,qb,n1,n2,n3,Tpx,Tpy,Tpz:"<<endl;
	for(int i=0;i<test_num;i++){
		outfile<<qa[i]<<' '<<qb[i]<<' '<<n1[i]<<' '<<n2[i]<<' '<<n3[i]<<' '<<Tpx[i]<<' '<<Tpy[i]<<' '<<Tpz[i]<<endl;
	}
	outfile<<"initial x:";
	for(int i=0;i<7;i++){
		outfile<<x[i]<<' ';
	}
	outfile.close();
}
void store_calibration_result(char *filename,double *x,int k,double adjustment1,double adjustment2,double time){
	ofstream outfile(filename,ios::out|ios::app);
	outfile<<endl;
	outfile<<"calibration result:"<<endl;
	outfile<<"fitted value:";
	for(int i=0;i<7;i++){
		outfile<<x[i]<<' ';
	}
	outfile<<endl;
	outfile<<"k:"<<k<<endl;
	outfile<<"judgement1 and judgement2:"<<adjustment1<<' '<<adjustment2<<endl;
	outfile<<"calibration time:"<<time<<"ms"<<endl;
	outfile.close();
}
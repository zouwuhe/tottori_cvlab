#include <fstream>
using namespace std;
void store_calibration_result(string filename,double *x,int k,double adjustment1,double adjustment2,double time){
	ofstream outfile(filename,ios::out|ios::app);
/*	for(int i=0;i<7;i++){
		outfile<<x[i]<<' ';
	}
	outfile<<endl;*/
	outfile<<"step:"<<k<<", ";
	outfile<<"judgement1 and judgement2:"<<adjustment1<<' '<<adjustment2<<", ";
	outfile<<"calibration time:"<<time<<"ms"<<endl;
	outfile.close();
}
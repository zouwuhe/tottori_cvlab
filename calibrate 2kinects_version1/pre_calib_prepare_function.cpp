//observed quantity: fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz;  function comes from maple: two kinect calibration without overlapped views2.mw
//parameter:a,b,c,d corresponding to x[0],x[1],x[2],x[3];
#include<math.h>
double pre_f_objective(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=10;
	double Tx=0;
	double Ty=0;
	double Tz=0;
	double a=x[0];
	double b=x[1];
	double c=x[2];
	double d=x[3];
	double cg = pow((-0.1e1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (0.1e1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d, 0.2e1) + lambda * pow(a * a + b * b + c * c + d * d - 0.1e1, 0.2e1);
	return cg;
}
double pre_pd_f_a(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=10;
	double Tx=0;
	double Ty=0;
	double Tz=0;
	double a=x[0];
	double b=x[1];
	double c=x[2];
	double d=x[3];
	double cg3 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (2 * a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - 2 * d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2)) + (1 / fy * qb - cy / fy) * (2 * a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - 2 * b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) - 2 * a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + 2 * c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * a;
	return cg3;
}
double pre_pd_f_b(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=10;
	double Tx=0;
	double Ty=0;
	double Tz=0;
	double a=x[0];
	double b=x[1];
	double c=x[2];
	double d=x[3];
	double cg4 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (-2 * b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - 2 * c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2)) + (1 / fy * qb - cy / fy) * (a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - 2 * c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) - a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + 2 * b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + 2 * d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * b;
	return cg4;
}
double pre_pd_f_c(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=10;
	double Tx=0;
	double Ty=0;
	double Tz=0;
	double a=x[0];
	double b=x[1];
	double c=x[2];
	double d=x[3];
	double cg5 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - 2 * b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - 2 * c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2)) + (1 / fy * qb - cy / fy) * (-b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - 2 * c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - 2 * d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) - a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + 2 * d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * c;
	return cg5;
}
double pre_pd_f_d(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=10;
	double Tx=0;
	double Ty=0;
	double Tz=0;
	double a=x[0];
	double b=x[1];
	double c=x[2];
	double d=x[3];
	double cg6 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - 2 * b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) + (1 / fy * qb - cy / fy) * (a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - 2 * c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - 2 * d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) + 2 * b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + 2 * d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * d;
	return cg6;
}
double pre_max(double a1,double a2,double a3,double a4){
	double result=a1;
	if(a2>result)
		result=a2;
	if(a3>result)
		result=a3;
	if(a4>result)
		result=a4;
	return result;
}
double pre_max_x_y(double x,double y){
	double result=x;
	if(y>result)
		result=y;
	return result;
}
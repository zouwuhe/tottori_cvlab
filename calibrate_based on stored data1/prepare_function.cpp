#include<math.h>
//observed quantity: fx,fy,cx,cy,qa,qb,n1,n2,n3,Tpx,Tpy,Tpz;  function comes from maple: two kinect calibration without overlapped views2.mw
//parameter:Tx,Ty,Tz,a,b,c,d corresponding to x[0],x[1]... ...x[6];
double f_objective(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg = pow((-0.1e1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (0.1e1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d, 0.2e1) + lambda * pow(a * a + b * b + c * c + d * d - 0.1e1, 0.2e1);
	return cg;
}
double pd_f_Tx(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg0 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * ((d * n3 + c * n2) * a - (-d * n2 + c * n3) * b - (b * n3 - a * n2) * c - (-b * n2 - a * n3) * d) + (1 / fy * qb - cy / fy) * ((-b * n2 - a * n3) * a - (a * n2 - b * n3) * b - (-d * n2 + c * n3) * c - (-c * n2 - d * n3) * d) - (a * n2 - b * n3) * a + (b * n2 + a * n3) * b + (d * n3 + c * n2) * c + (-d * n2 + c * n3) * d);
	return cg0;
}
double pd_f_Ty(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg1 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * ((a * n3 - c * n1) * a - (-b * n3 + d * n1) * b - (a * n1 + c * n3) * c - (b * n1 + d * n3) * d) + (1 / fy * qb - cy / fy) * ((b * n1 + d * n3) * a - (-c * n3 - a * n1) * b - (-b * n3 + d * n1) * c - (c * n1 - a * n3) * d) - (-c * n3 - a * n1) * a + (-d * n3 - b * n1) * b + (a * n3 - c * n1) * c + (-b * n3 + d * n1) * d);
	return cg1;
}
double pd_f_Tz(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg2 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * ((-a * n2 - d * n1) * a - (b * n2 - c * n1) * b - (-b * n1 - c * n2) * c - (a * n1 - d * n2) * d) + (1 / fy * qb - cy / fy) * ((a * n1 - d * n2) * a - (c * n2 + b * n1) * b - (b * n2 - c * n1) * c - (d * n1 + a * n2) * d) - (c * n2 + b * n1) * a + (d * n2 - a * n1) * b + (-a * n2 - d * n1) * c + (b * n2 - c * n1) * d);
	return cg2;
}
double pd_f_a(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg3 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (2 * a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - 2 * d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2)) + (1 / fy * qb - cy / fy) * (2 * a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - 2 * b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) - 2 * a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + 2 * c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * a;
	return cg3;
}
double pd_f_b(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg4 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (-2 * b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - 2 * c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2)) + (1 / fy * qb - cy / fy) * (a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - 2 * c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) - a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + 2 * b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + 2 * d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * b;
	return cg4;
}
double pd_f_c(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg5 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - 2 * b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - 2 * c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2)) + (1 / fy * qb - cy / fy) * (-b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - 2 * c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - 2 * d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) - a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + 2 * d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * c;
	return cg5;
}
double pd_f_d(double fx,double fy,double cx,double cy,int qa,int qb,double n1,double n2,double n3,double Tpx,double Tpy,double Tpz,double *x){
	double lambda=1;
	double Tx=x[0];
	double Ty=x[1];
	double Tz=x[2];
	double a=x[3];
	double b=x[4];
	double c=x[5];
	double d=x[6];
	double cg6 = 2 * ((-1 / fx * qa + cx / fx) * ((a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + c * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) - d * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + a * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) + (1 / fy * qb - cy / fy) * ((d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a - (c * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) - b * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b - (c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - b * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c - (c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) - (-c * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) + b * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + a * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2)) * a + (d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + b * ((-Ty + Tpy) * n1 + (Tx - Tpx) * n2) + a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3)) * b + (d * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) + a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3)) * c + (d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) * d) * ((-1 / fx * qa + cx / fx) * (a * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) - 2 * b * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - d * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - a * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) + d * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) + (1 / fy * qb - cy / fy) * (a * ((-Tz + Tpz) * n2 + (Ty - Tpy) * n3) - 2 * c * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - 2 * d * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3) - a * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3)) + 2 * b * (-(-Tz + Tpz) * n2 - (Ty - Tpy) * n3) + c * (-(Tz - Tpz) * n1 - (-Tx + Tpx) * n3) + 2 * d * (-(-Ty + Tpy) * n1 - (Tx - Tpx) * n2) - c * ((Tz - Tpz) * n1 + (-Tx + Tpx) * n3)) + 4 * lambda * (a * a + b * b + c * c + d * d - 1) * d;
	return cg6;
}
double max(double a1,double a2,double a3,double a4,double a5,double a6,double a7){
	double result=a1;
	if(a2>result)
		result=a2;
	if(a3>result)
		result=a3;
	if(a4>result)
		result=a4;
	if(a5>result)
		result=a5;
	if(a6>result)
		result=a6;
	if(a7>result)
		result=a7;
	return result;
}
double max_x_y(double x,double y){
	double result=x;
	if(y>result)
		result=y;
	return result;
}

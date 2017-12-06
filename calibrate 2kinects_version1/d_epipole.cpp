#include <math.h>
double d_epipole(double cx,double cy,double fx,double fy, double h,double T1,double T2,double T3,double a,double b,double c,double d,double Tpx,double Tpy,double Tpz,double n1,double n2,double n3,double a0,double b0){
	double A=((T2 - Tpy) * n3 - (T3 - Tpz) * n2) * (-a * a / fx - b * b / fx + c * c / fx + d * d / fx) + ((T3 - Tpz) * n1 - (T1 - Tpx) * n3) * (2 * d / fx * a - 2 * c / fx * b) 
		+ ((T1 - Tpx) * n2 - (T2 - Tpy) * n1) * (-2 * c / fx * a - 2 * d / fx * b);
	double B=((T2 - Tpy) * n3 - (T3 - Tpz) * n2) * (-2 * d / fy * a - 2 * c / fy * b) + ((T3 - Tpz) * n1 - (T1 - Tpx) * n3) * (-a * a / fy + b * b / fy - c * c / fy + d * d / fy) 
		+ ((T1 - Tpx) * n2 - (T2 - Tpy) * n1) * (2 * b / fy * a - 2 * d / fy * c);
	double F=((T2 - Tpy) * n3 - (T3 - Tpz) * n2) * ((a * (cx - a0) / fx + d * (b0 - cy) / fy + c) * a + (b * (cx - a0) / fx + c * (b0 - cy) / fy - d) * b + (a + b * (b0 - cy) / fy 
		- c * (cx - a0) / fx) * c + (-b + a * (b0 - cy) / fy - d * (cx - a0) / fx) * d) + ((T3 - Tpz) * n1 - (T1 - Tpx) * n3) * ((-b + a * (b0 - cy) / fy - d * (cx - a0) / fx) * a 
		+ (c * (cx - a0) / fx - a - b * (b0 - cy) / fy) * b + (b * (cx - a0) / fx + c * (b0 - cy) / fy - d) * c + (-a * (cx - a0) / fx - c - d * (b0 - cy) / fy) * d)
		+ ((T1 - Tpx) * n2 - (T2 - Tpy) * n1) * ((c * (cx - a0) / fx - a - b * (b0 - cy) / fy) * a + (-a * (b0 - cy) / fy + b + d * (cx - a0) / fx) * b + (a * (cx - a0) / fx 
		+ d * (b0 - cy) / fy + c) * c + (b * (cx - a0) / fx + c * (b0 - cy) / fy - d) * d);
	double d_epipole=abs(F)/sqrt(pow(A,2.0)+pow(B,2.0));
	return d_epipole;
}
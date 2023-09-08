#ifndef VELOCITY_H
#define VELOCITY_H
#define DEG_TO_RAD 0.01745555555
#include <BasicLinearAlgebra.h>

using namespace BLA;
class Altitude
{
public:
	Altitude(float*, float*, float*, float*);
	void velocity_real(double, double, double, double, double);
	void Altitude_Kal(float, double);
private:
	double  velocity_prev, AccZvelocity_prev;
	double period = 0.01;
	BLA::Matrix<2, 2>F = { 1, 0.01,0 ,1 };
	BLA::Matrix<2, 1>G = { 0.5*0.01*0.01, 0.01 };
	BLA::Matrix<2, 2>P = { 0,0,0,0 };
	BLA::Matrix<2, 2>Q = { 2.5e-7 ,0.00005, 0.00005, 0.01 };
	BLA::Matrix<2, 1>S;
	BLA::Matrix<1, 2>H = { 1,0 };
	BLA::Matrix<2, 2>I = { 1,0,0,1 };
	BLA::Matrix<1, 1>Acc;
	BLA::Matrix<2, 1>K;
	BLA::Matrix<1, 1>R = { 30 * 30 };
	BLA::Matrix<1, 1>L;
	BLA::Matrix<1, 1>M;

	//F = { 1, 0.01,0 ,1 };
	//Matrix<1, 1>G = { 0.5*0.01*0.01, 0.01 };
// Matrix<2, 2>P = { 0,0,0,0 };
// Matrix<1, 2>H = { 1,0 };
// Matrix<2, 2>I = { 1,0,0,1 };
// Matrix<1, 1>R = { 30 * 30 };
// Matrix<1, 2>S = { 0 ,0 };
	float* myAccZ;
	float* myAltitudeKal;
	float* MyVelocityKal;
	float* myVelocity;
};


#endif // !VELOCITY_H

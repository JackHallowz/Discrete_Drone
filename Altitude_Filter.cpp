
#include "Altitude_Filter.h"
#include <BasicLinearAlgebra.h>

Altitude::Altitude(float* accZ, float* AltitudeKal, float* VelocityKal, float* Velocity)
{
	myAccZ = accZ;
	myAltitudeKal = AltitudeKal;
	MyVelocityKal = VelocityKal;
	myVelocity = Velocity;


	
}
void Altitude::velocity_real(double roll, double pitch, double accX, double accY, double accZ)
{
	double accZ_real = -sin(pitch * DEG_TO_RAD)* accX + cos(pitch*DEG_TO_RAD) * sin(roll*DEG_TO_RAD) * accY
		+ cos(pitch*DEG_TO_RAD) * cos(roll*DEG_TO_RAD) * accZ;
	accZ_real = (accZ_real - 1)*9.81 * 100;
	double AccZvelocity = AccZvelocity + period * accZ_real;
	*myVelocity = (float) AccZvelocity;
	*myAccZ = (float)accZ_real;
}

void Altitude::Altitude_Kal(float accZ, double baroAltitude)
{
	Acc = { accZ };
	S = F * S + G * Acc;
	P = F * P * ~F + Q;
	L = H * P * ~H + R;
	Invert(L);
	K = P * ~H * L;
	M = (float)baroAltitude;
	S = S + K * (M - H * S);
	*myAltitudeKal = S(0, 0);
	*MyVelocityKal = S(1, 0);
	P = (I - K * H) * P;
	



}
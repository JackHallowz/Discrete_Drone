 #include "PID_dis.h"
#include "Arduino.h"

dis_PID::dis_PID(double* Input, double* Output, double* Setpoint, double Kp, double Ki, double Kd)
{
	Myoutput = Output;
	Myinput = Input;
	Mysetpoint = Setpoint;
	//dis_PID::setoutputlimits(-1000, 1000);
	lastout, error_1, error_2 = 0;
	dis_PID::SetTunnings(Kp, Ki, Kd);
}

void dis_PID::compute()
{
	double input = *Myinput;
	error = *Mysetpoint - input;
	alpha = 2 * Myperiod * kp + ki * Myperiod * Myperiod + 2 * kd;
	beta = Myperiod * Myperiod*ki - 4 * kd - 2 * Myperiod*kp;
	gamma = 2 * kd;
	output = (alpha * error + beta * error_1 + gamma * error_2 + 2 * Myperiod * lastout) / (2 * Myperiod);
	//if (output > outMax) output = outMax;
	//else if (output < outMin) output = outMin;
	lastout = output;
	*Myoutput = output;
	error_2 = error_1;
	error_1 = error;
}

void dis_PID::setoutputlimits(double Min, double Max)
{
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;
}

void dis_PID::SetTunnings(double Kp, double Ki, double Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0) return;
	kp = Kp;
	ki = Ki;
	kd = Kd;
}

void dis_PID::setKp(double nKp)
{
	kp = nKp;
}

void dis_PID::setKi(double nKi)
{
	ki = nKi;
}

void dis_PID::setKd(double nKd)
{
	kd = nKd;
}

double dis_PID::getKp() { return kp; };
double dis_PID::getKi() { return ki; };
double dis_PID::getKd() { return kd; };


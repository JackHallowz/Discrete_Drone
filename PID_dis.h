#pragma once
#ifndef PID_DIS_H
#define PID_DIS_H
#include "Arduino.h"

class dis_PID
{
public:
	dis_PID(double*, double*, double*,double, double, double);

	void setoutputlimits(double, double);
	void compute();
	void SetTunnings(double, double, double);
	void setKp(double);
	void setKi(double);
	void setKd(double);
	double getKp();
	double getKi();
	double getKd();

private:
	double kp;
	double ki;
	double kd;
	double alpha, gamma, beta;
	double error,output;

	double* Myinput;
	double* Mysetpoint;
	double* Myoutput;

	const double Myperiod = 0.01;

	double outMin, outMax;
	double lastout=0, error_1=0, error_2=0;

};
#endif // PID_DIS_H

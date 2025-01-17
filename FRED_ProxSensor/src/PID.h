#ifndef PID_H
#define PID_H

// Define PID Functions
double PID_calc(double error);
double GetPID(double RatPos, double RobPos);
double getPIDSpeed(double RatPos, double RobPos, double *errorLast, double *milliLast);
bool isStopped(float ratPos, float *lastRatPosition, int *ratStopTime);
int rewardAmount(float ratPos, float cuePos);

#endif
#include "pid.h"
#include <iostream>

PID_State pid_iterate(PID_Calibration calibration, PID_State state)
{
    // calculate difference between desired and actual values (the error)
    double error = state.target - state.actual;
    // calculate and update integral
    state.integral += (error * state.time_delta);
    // calculate derivative
    double derivative = (error - state.previous_error) / state.time_delta;
    // calculate output value according to algorithm
    state.output = ((calibration.kp * error) + (calibration.ki * state.integral) + (calibration.kd * derivative));
    // update state.previous_error to the error value calculated on this iteration
    state.previous_error = error;
    // return the state struct reflecting the calculations
    return state;
}

// ��ʼ��kp,ki,kd(��Ҫ��������Σ��ں���set_calibration()�������)
void init_calibration(PID_Calibration calibration)
{
    calibration.kp = 0; // ��
    calibration.ki = 0;
    calibration.kd = 0;
}

// �趨kp,ki,kd(�ڴ˵���)
void set_calibration(PID_Calibration calibration)
{
    calibration.kp = 0;
    calibration.ki = 0;
    calibration.kd = 0;
}

PID_Calibration calibration;

int main()
{
    init_calibration(calibration); // ��ʼ��kp,ki,kdΪ0
    set_calibration(calibration);  // �趨kp,ki,kd
    while (1)
    {
    }
}
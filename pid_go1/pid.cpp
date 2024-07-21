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

// 初始化kp,ki,kd(不要在这里调参，在函数set_calibration()里面调参)
void init_calibration(PID_Calibration calibration)
{
    calibration.kp = 0; // 勿动
    calibration.ki = 0;
    calibration.kd = 0;
}

// 设定kp,ki,kd(在此调参)
void set_calibration(PID_Calibration calibration)
{
    calibration.kp = 0;
    calibration.ki = 0;
    calibration.kd = 0;
}

PID_Calibration calibration;

int main()
{
    init_calibration(calibration); // 初始化kp,ki,kd为0
    set_calibration(calibration);  // 设定kp,ki,kd
    while (1)
    {
    }
}
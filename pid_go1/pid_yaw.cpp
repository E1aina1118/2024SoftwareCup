#include <iostream>
#include "pid.h"
#include <math.h>

PID_Calibration calibration;//kp,ki,kd参数
PID_State pid_state;// 状态变量

void set_calibration(PID_Calibration *calibration, double kp, double ki, double kd)
{
    calibration->kp = kp;
    calibration->ki = ki;
    calibration->kd = kd;
}

set_calibration(calibration,5,0,0);

PID_State pid_iterate(PID_Calibration calibration, PID_State state, double error)
{
    state.time_delta = 0.003;
    // double error = state.target - state.actual;
    if (error < -M_PI)
    {
        error = 2 * M_PI + error;
    }
    else if (error > M_PI)
    {
        error = error - 2 * M_PI;
    }
    state.integral += (error * state.time_delta);
    double derivative = (error - state.previous_error) / state.time_delta;
    state.output = ((calibration.kp * error) + (calibration.ki * state.integral) + (calibration.kd * derivative));
    state.previous_error = error;
    return state;
}
// 以下放进循环函数
pid_state = pid_iterate(calibration,pid_state,error);
cmd.yawSpeed = pid_state.output;

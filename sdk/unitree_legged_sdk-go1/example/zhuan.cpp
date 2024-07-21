#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <math.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include "pid.h"
using namespace UNITREE_LEGGED_SDK;
double current = 0;
int motion = 1;
PID_State pid_iterate(PID_Calibration calibration, PID_State state)
{
    // calculate difference between desired and actual values (the error)
    state.time_delta = 0.002;
    double error = state.target - state.actual;
    if (error < -M_PI)
    {
        error = 2 * M_PI + error;
    }
    else if (error > M_PI)
    {
        error = error - 2 * M_PI;
    }
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
PID_Calibration calibration_1;
PID_State pid_state;

// 设定kp,ki,kd数值
void set_calibration(PID_Calibration *calibration, double kp, double ki, double kd)
{
    calibration->kp = kp;
    calibration->ki = ki;
    calibration->kd = kd;
}
class Custom
{
public:
    Custom(uint8_t level) : safe(LeggedType::Go1),
                            udp(level, 8090, "192.168.123.161", 8082)
    {
        udp.InitCmdData(cmd);
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
};

void Custom::UDPRecv()
{
    udp.Recv();
}

void Custom::UDPSend()
{
    udp.Send();
}

double compute_error(double target, double actual)
{
    double error = target - s;
    if (error < -M_PI)
    {
        error = 2 * M_PI + error;
    }
    else if (error > M_PI)
    {
        error = error - 2 * M_PI;
    }
    return error
}

int status(double target, double s)
{
    double error = compute_error(target, s);
    if (fabs(error) <= M_PI / 180 * 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void Custom::RobotControl()
{

    set_calibration(&calibration_1, 6, 0, 0);
    motiontime += 2;
    udp.GetRecv(state);
    //   printf("%d   %f\n", motiontime, state.imu.quaternion[2]);
    cmd.mode = 0; // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd.gaitType = 0;
    cmd.speedLevel = 0;
    cmd.footRaiseHeight = 0;
    cmd.bodyHeight = 0;
    cmd.euler[0] = 0;
    cmd.euler[1] = 0;
    cmd.euler[2] = 0;
    cmd.velocity[0] = 0.0f;
    cmd.velocity[1] = 0.0f;
    cmd.yawSpeed = 0.0f;
    cmd.reserve = 0;
    if (motiontime < 1000)
    {
        current = state.imu.rpy[2];
        pid_state.target = current;
    }
    if (motiontime >= 1000)
    {
        cmd.mode = 2;
        current = state.imu.rpy[2];
        pid_state.actual = state.imu.rpy[2];
        if(motion == 2)
        {
            pid_state.target = current + M_PI/2;
            while(1)
            {
                pid_state.actual = state.imu.rpy[2];
                pid_state = pid_iterate(calibration_1, pid_state);
                cmd.yawSpeed = pid_state.output;
                cmd.velocity[0] = 0.3;
                udp.SetSend(cmd);
                if(status(pid_state.target,pid_state.actual) == 1) break;
            }
        }
        else if (motion == 3)
        {
            pid_state.target = current - M_PI/2;
            while(1)
            {
                pid_state.actual = state.imu.rpy[2];
                pid_state = pid_iterate(calibration_1, pid_state);
                cmd.yawSpeed = pid_state.output;
                cmd.velocity[0] = 0.15;
                udp.SetSend(cmd);
                if(status(pid_state.target,pid_state.actual) == 1) break;
            }
        }
        else
        {
            cmd.yawSpeed = 0;
            cmd.velocity[0] = 0;
            udp.SetSend(cmd);
        }
        // pid_state = pid_iterate(calibration_1, pid_state);
        // cmd.yawSpeed = pid_state.output;
    }
    // printf("imu.rpy[2](Yaw) = %f\n", state.imu.rpy[2]);
    // printf("motiontime = %d\n", motiontime);
    // printf("output = %lf\n", pid_state.output);
    // printf("velocity = %lf\n", state.velocity[0]);
    // printf("\n");
    // if(motiontime>30000)
    // {
    //     cmd.mode = 0;
    //     cmd.velocity[0] = 0;
    // }
    // 显示当前yaw数值：
    // udp.SetSend(cmd);
}
int main(int argc, char **argv)
{
    std::cout << "Communication level is set to HIGH-level." << std::endl
              << "WARNING: Make sure the robot is standing on the ground." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    Custom custom(HIGHLEVEL);
    LoopFunc loop_udpSend("udp_send", custom.dt, 3, boost::bind(&Custom::UDPSend, &custom));
    LoopFunc loop_udpRecv("udp_recv", custom.dt, 3, boost::bind(&Custom::UDPRecv, &custom));
    LoopFunc loop_control("control", custom.dt, 3, boost::bind(&Custom::RobotControl, &custom));
    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();
    while (1)
    {

        sleep(10);
    };
    return 0;
}

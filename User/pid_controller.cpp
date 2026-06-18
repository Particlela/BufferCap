/**
 * @file    pid_controller.cpp
 * @brief   位置式PI控制器类实现
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

#include "pid_controller.hpp"

namespace supercap {

PIDController::PIDController(float kp, float ki, float out_max, float iout_max, float out_min, float iout_min)
    : kp(kp), ki(ki), out_max(out_max), out_min(out_min), iout_max(iout_max), iout_min(iout_min)
{
    reset();
}

void PIDController::init(float kp_, float ki_, float out_max_, float iout_max_, float out_min_, float iout_min_)
{
    kp = kp_;
    ki = ki_;
    out_max = out_max_;
    out_min = out_min_;
    iout_max = iout_max_;
    iout_min = iout_min_;
    reset();
}

void PIDController::reset()
{
    pout = 0.0f;
    iout = 0.0f;
    out = 0.0f;
    error[0] = 0.0f;
    error[1] = 0.0f;
}

float PIDController::compute(float ref, float set)
{
    error[1] = error[0];
    error[0] = set - ref;
    pout = kp * error[0];
    iout += ki * error[0];

    iout = (iout > iout_max) ? iout_max : iout;
    iout = (iout < iout_min) ? iout_min : iout;

    out = pout + iout;
    out = (out > out_max) ? out_max : out;
    out = (out < out_min) ? out_min : out;

    return out;
}

} // namespace supercap

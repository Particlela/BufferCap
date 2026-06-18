/**
 * @file    pid_controller.hpp
 * @brief   位置式PI控制器类
 */
#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#ifdef __cplusplus
}
#endif

namespace supercap {

class PIDController {
public:
    PIDController() = default;
    PIDController(float kp, float ki, float out_max, float iout_max, float out_min, float iout_min);

    void init(float kp, float ki, float out_max, float iout_max, float out_min, float iout_min);
    float compute(float ref, float set);
    void reset();

    // 参数
    float kp = 0.0f;
    float ki = 0.0f;

    // 输出
    float pout = 0.0f;
    float iout = 0.0f;
    float out = 0.0f;

    // 限幅
    float out_max = 0.0f;
    float out_min = 0.0f;
    float iout_max = 0.0f;
    float iout_min = 0.0f;

    float error[2] = {0.0f, 0.0f};
};

} // namespace supercap

#endif // PID_CONTROLLER_HPP

/**
 * @file    buckboost.hpp
 * @brief   Buck-Boost 变换器 PWM 控制类
 */
#ifndef BUCKBOOST_HPP
#define BUCKBOOST_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "hrtim.h"

#ifdef __cplusplus
}
#endif

namespace supercap {

class BuckBoostConverter {
public:
    void update(float volt_ratio);

    float buck_duty() const { return buck_duty_; }
    float boost_duty() const { return boost_duty_; }

private:
    static constexpr float k_max_volt_ratio = 1.32f;
    static constexpr float k_min_volt_ratio = 0.05f;
    static constexpr float k_max_duty = 0.96f;
    static constexpr float k_min_active_duty = 0.04f;
    static constexpr float k_correction = 0.032f;

    float buck_duty_ = 0.0f;
    float boost_duty_ = 0.0f;
};

} // namespace supercap

#endif // BUCKBOOST_HPP

/**
 * @file    buckboost.cpp
 * @brief   Buck-Boost 变换器 PWM 控制类实现
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "hrtim.h"

#ifdef __cplusplus
}
#endif

#include "buckboost.hpp"

namespace supercap {

void BuckBoostConverter::update(float volt_ratio)
{
    if (k_max_volt_ratio < volt_ratio) {
        volt_ratio = k_max_volt_ratio;
    }
    if (k_min_volt_ratio > volt_ratio) {
        volt_ratio = k_min_volt_ratio;
    }

    if (volt_ratio < 1.0f) {
        buck_duty_ = 1.0f - volt_ratio + k_correction;
        boost_duty_ = k_min_active_duty;
    } else {
        buck_duty_ = k_min_active_duty;
        boost_duty_ = 1.0f - (1.0f / volt_ratio) + k_correction;
    }

    if (buck_duty_ > k_max_duty) {
        buck_duty_ = k_max_duty;
    }
    if (boost_duty_ > k_max_duty) {
        boost_duty_ = k_max_duty;
    }

    HRTIM1->sMasterRegs.MCMP1R = (HRTIM1->sMasterRegs.MPER * (1.0f - buck_duty_)) / 2.0f;
    HRTIM1->sMasterRegs.MCMP2R = (HRTIM1->sMasterRegs.MPER * (1.0f + buck_duty_)) / 2.0f;
    HRTIM1->sMasterRegs.MCMP3R = (HRTIM1->sMasterRegs.MPER * (1.0f - boost_duty_)) / 2.0f;
    HRTIM1->sMasterRegs.MCMP4R = (HRTIM1->sMasterRegs.MPER * (1.0f + boost_duty_)) / 2.0f;
}

} // namespace supercap

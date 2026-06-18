/**
 * @file    control_task.hpp
 * @brief   超级电容主控制任务类
 */
#ifndef CONTROL_TASK_HPP
#define CONTROL_TASK_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
}
#endif

#include "adc_sampler.hpp"
#include "pid_controller.hpp"
#include "buckboost.hpp"
#include "super_cap_comm.hpp"
#include "debug_interface.hpp"

namespace supercap {

class ControlTask {
public:
    static constexpr uint16_t k_board_id = 1;
    static constexpr uint32_t k_soft_start_time = SOFT_START_TIME;
    static constexpr float k_cap_v_max = 25.5f;
    static constexpr float k_vin_low_threshold = 16.5f;
    static constexpr float k_vin_high_threshold = 29.0f;
    static constexpr float k_vin_recover_low = 17.5f;
    static constexpr float k_vin_recover_high = 27.0f;
    static constexpr float k_uvp_trigger = 10.0f;
    static constexpr float k_uvp_release = 12.0f;
    static constexpr uint32_t k_can_timeout_ms = 500;

    void init();
    void dispatch(TIM_HandleTypeDef *htim);

    // 控制状态访问
    float power_set() const { return power_set_; }
    float power_set_target() const { return power_set_target_; }
    float cap_v_max() const { return cap_v_max_; }
    float volt_ratio() const { return volt_ratio_; }
    float ploop_ref() const { return ploop_ref_; }
    float cloop_ref() const { return cloop_ref_; }
    bool uvp() const { return uvp_; }
    int can_disable_cnt() const { return can_disable_cnt_; }
    bool can_disable_flag() const { return can_disable_flag_; }
    bool restart_flag() const { return restart_flag_; }

    const AdcSampler& sampler() const { return sampler_; }
    const SuperCapComm& comm() const { return *comm_; }

private:
    void onTim2();
    void onTim3();
    void onTim4();

    void sampleAndFilter();
    void softStart();
    void computeControlLoop();
    void updatePwm();
    void checkProtection();

    AdcSampler sampler_;
    BuckBoostConverter converter_;
    PIDController vloop_;
    PIDController ploop_;
    SuperCapComm *comm_ = nullptr;

    float power_set_ = 0.0f;
    float power_set_target_ = 0.0f;
    float cap_v_max_ = k_cap_v_max;
    float volt_ratio_ = 0.0f;
    float ploop_ref_ = 0.0f;
    float vloop_ref_ = 0.0f;
    float cloop_ref_ = 0.0f;
    bool uvp_ = false;
    bool restart_flag_ = false;
    int can_disable_cnt_ = 0;
    bool can_disable_flag_ = false;
    uint16_t power_start_tick_ = 0;
    uint16_t can_div_cnt_ = 0;

    DebugInterface debugger_;
};

} // namespace supercap

#endif // CONTROL_TASK_HPP

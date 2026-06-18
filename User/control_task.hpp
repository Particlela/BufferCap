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

enum class BufferCapState : uint8_t {
    kIdle = 0,         // PWM关闭，等待条件
    kCharging,         // 功率环充电
    kVoltageReg,       // 电压环稳压
    kDischarging,      // 主动放电
};

class ControlTask {
public:
    static constexpr uint16_t k_board_id = 1;
    static constexpr uint32_t k_soft_start_time = SOFT_START_TIME;

    // 电容电压相关
    static constexpr float k_vcap_target = 28.5f;            // 电容目标电压
    static constexpr float k_vcap_max = 29.5f;               // 电容最大允许电压
    static constexpr float k_vcap_danger_low = 6.0f;        // 电容放电危险下限

    // 母线电压阈值（滞回）
    static constexpr float k_vin_charge_threshold = 22.8f;   // 母线充电阈值
    static constexpr float k_vin_discharge_threshold = 22.0f;// 母线放电阈值
    static constexpr float k_vin_reg_target = 22.5f;         // 放电时母线稳压目标

    // 充放电限制
    static constexpr float k_p_charge_max = 190.0f;          // 最大充电功率

    // 保护阈值
    static constexpr float k_vin_low_threshold = 15.0f;      // 母线欠压保护
    static constexpr float k_vin_high_threshold = 29.0f;     // 母线过压保护
    static constexpr float k_vin_recover_low = 16.0f;        // 母线恢复阈值
    static constexpr float k_vin_recover_high = 27.0f;       // 母线恢复阈值
    static constexpr float k_vcap_overvolt = 29.0f;          // 电容过压保护

    static constexpr uint32_t k_can_timeout_ms = 500;

    void init();
    void dispatch(TIM_HandleTypeDef *htim);

    // 控制状态访问
    BufferCapState state() const { return state_; }
    float bus_power_target() const { return bus_power_target_; }
    float power_set() const { return power_set_; }
    float cap_v_max() const { return cap_v_max_; }
    float volt_ratio() const { return volt_ratio_; }
    float ploop_ref() const { return ploop_ref_; }
    float vloop_ref() const { return vloop_ref_; }
    float cloop_ref() const { return cloop_ref_; }
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
    PIDController vloop_;          // 电容电压环
    PIDController ploop_;          // 功率环
    PIDController bus_vloop_;      // 母线电压环（放电时稳压 Vin）
    SuperCapComm *comm_ = nullptr;

    BufferCapState state_ = BufferCapState::kIdle;
    float bus_power_target_ = 0.0f;
    float power_set_ = 0.0f;
    float cap_v_max_ = k_vcap_target;
    float volt_ratio_ = 0.0f;
    float ploop_ref_ = 0.0f;
    float vloop_ref_ = 0.0f;
    float cloop_ref_ = 0.0f;
    bool restart_flag_ = false;
    int can_disable_cnt_ = 0;
    bool can_disable_flag_ = false;
    uint16_t power_start_tick_ = 0;
    uint16_t can_div_cnt_ = 0;

    DebugInterface debugger_;
};

} // namespace supercap

#endif // CONTROL_TASK_HPP

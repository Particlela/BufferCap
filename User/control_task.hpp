/**
 * @file    control_task.hpp
 * @brief   Super capacitor control task.
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
    kIdle = 0,
    kPower,
    kVoltageReg,
    kDischarging,
};

class ControlTask {
public:
    static constexpr uint16_t k_board_id = 0;
    static constexpr uint32_t k_soft_start_time = SOFT_START_TIME;

    static constexpr float k_vcap_high = 25.0f;
    static constexpr float k_vcap_low  = 11.0f; 
    static constexpr float k_vcap_max  = 26.0f;
    static constexpr float k_vcap_min  = 9.0f; 
    static constexpr float k_vcap_clamp_exit = 9.5f;

    static constexpr float k_vin_charge_threshold    = 22.8f;
    static constexpr float k_vin_discharge_threshold = 22.1f;
    static constexpr float k_vin_reg_target          = 22.2f;

    static constexpr float k_p_charge_max       = 190.0f;
    static constexpr float k_bus_power_min      = 10.0f;
    static constexpr float k_vcap_current_limit = 50.0f;
    static constexpr float k_power_ramp_rate    = 10000.0f;
    static constexpr float k_charge_power_margin = 10.0f;

    static constexpr float k_soft_start_vcap_threshold = k_vcap_min;
    static constexpr float k_soft_start_vcap_power_limit = 35.0f;

    static constexpr float k_vcap_high_hysteresis = 0.5f;
    static constexpr float k_vcap_low_hysteresis  = 1.0f;
    static constexpr float k_vcap_target_tolerance = 0.1f;
    static constexpr float k_vcap_high_tolerance = 0.1f;
    static constexpr float k_vcap_low_tolerance = 0.8f;

    static constexpr float k_ffd_max = 1.25f;

    static constexpr float k_vin_low_threshold  = 15.0f;
    static constexpr float k_vin_high_threshold = 28.0f;
    static constexpr float k_vin_recover_low    = 16.0f;
    static constexpr float k_vin_recover_high   = 27.0f;

    static constexpr float k_cout_discharge_threshold  = 10.0f; 

    static constexpr uint32_t k_can_timeout_ms = 100;

    void init();
    void dispatch(TIM_HandleTypeDef *htim);

    BufferCapState state() const { return state_; }
    float bus_power_target() const { return bus_power_target_; }
    float power_set() const { return power_set_; }
    float cap_v_max() const { return cap_v_max_; }
    float volt_ratio() const { return volt_ratio_; }
    float ploop_ref() const { return ploop_ref_; }
    float vloop_ref() const { return vloop_ref_; }
    float cloop_ref() const { return cloop_ref_; }
    float vcap_reg_target() const { return vcap_reg_target_; }
    int can_disable_cnt() const { return can_disable_cnt_; }
    bool can_disable_flag() const { return can_disable_flag_; }
    bool restart_flag() const { return restart_flag_; }
    bool low_clamp_active() const { return low_clamp_active_; }
    bool can_charge_cap() const { return can_charge_cap_; }
    float pcap() const { return pcap_; }
    float load_power_est() const { return load_power_est_; }
    uint32_t work_tick() const { return work_tick_; }

    const AdcSampler& sampler() const { return sampler_; }
    const SuperCapComm& comm() const { return *comm_; }
    void debugSync();

private:
    static constexpr uint16_t k_debug_send_div = 100;

    void onTim2();
    void onTim3();
    void onTim4();

    void sampleAndFilter();
    void softStart();
    void computeControlLoop();
    void updatePwm();
    void checkProtection();
    void resetVoltageLoops();

    AdcSampler sampler_;
    BuckBoostConverter converter_;
    PIDController cap_high_vloop_;
    PIDController cap_low_vloop_;
    PIDController ploop_;
    PIDController bus_vloop_;
    SuperCapComm *comm_ = nullptr;

    BufferCapState state_ = BufferCapState::kIdle;
    float bus_power_target_ = 0.0f;
    float power_set_ = 0.0f;
    float cap_v_max_ = k_vcap_high;
    float volt_ratio_ = 0.0f;
    float ploop_ref_ = 0.0f;
    float vloop_ref_ = 0.0f;
    float cloop_ref_ = 0.0f;
    float vcap_reg_target_ = k_vcap_high;
    float pcap_ = 0.0f;
    float load_power_est_ = 0.0f;

    bool restart_flag_ = false;
    bool low_clamp_active_ = false;
    bool can_charge_cap_ = false;
    int can_disable_cnt_ = 0;
    bool can_disable_flag_ = false;
    uint16_t power_start_tick_ = 0;
    uint16_t can_div_cnt_ = 0;
    uint16_t debug_div_cnt_ = 0;
    uint32_t work_tick_ = 0;
    uint32_t work_tick_base_ = 0;
    DebugInterface debugger_;
};

} // namespace supercap

#endif // CONTROL_TASK_HPP

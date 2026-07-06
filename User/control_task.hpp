/**
 * @file    control_task.hpp
 * @brief   超级电容主控制任务类
 *
 * 状态机说明：
 *
 *  kIdle        — 平衡态 (volt_ratio = ffd_ratio)，不主动充放电
 *  kPower       — 恒输入功率模式 (功率环 + 电压环取小)
 *  kVoltageReg  — 电压环稳压 (稳压至 vcap_reg_target_，可为 k_vcap_high 或 k_vcap_low)
 *  kDischarging — 母线电压环放电，维持母线电压
 *
 * 全局保护逻辑 (在任何状态下均生效)：
 *  - vcap >= k_vcap_max  → 关断 PWM，进入 kIdle
 *  - vcap <= k_vcap_min  → 关断 PWM，进入 kIdle (启动充电模式除外)
 *  - vcap >= k_vcap_high → 切换 kVoltageReg，稳压至 k_vcap_high (防过充)
 *  - vcap <= k_vcap_low  → 切换 kVoltageReg，稳压至 k_vcap_low  (防过放)
 *
 * 启动充电模式 (startup_charging_)：
 *  - CAN 刚连上时若 vcap < k_vcap_min，允许恒功率模式充电 (豁免 k_vcap_min 关断)
 *  - 充至 vcap > k_vcap_low + k_vcap_low_hysteresis 后退出启动模式，进入正常工作
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

/// @brief 缓冲电容状态机状态
enum class BufferCapState : uint8_t {
    kIdle = 0,         ///< 平衡态，不主动充放电
    kPower,            ///< 恒输入功率模式 (功率环 + 电压环取小)
    kVoltageReg,       ///< 电压环稳压
    kDischarging,      ///< 主动放电支撑母线
};

class ControlTask {
public:
    static constexpr uint16_t k_board_id = 0;
    static constexpr uint32_t k_soft_start_time = SOFT_START_TIME;

    // ====== 电容电压阈值 ======
    /// 稳压上限：充电达到此值后切稳压，防止过充
    static constexpr float k_vcap_high = 25.0f;
    /// 稳压下限：放电降至此值后切稳压，防止过放
    static constexpr float k_vcap_low  = 11.0f;
    /// 硬关断上限：达到此值立即关断 PWM
    static constexpr float k_vcap_max  = 26.0f;
    /// 硬关断下限：达到此值立即关断 PWM (启动充电模式除外)
    static constexpr float k_vcap_min  = 8.0f;

    // ====== 母线电压阈值 (滞回) ======
    /// 母线高于此值时允许充电
    static constexpr float k_vin_charge_threshold    = 22.8f;
    /// 母线低于此值时触发放电
    static constexpr float k_vin_discharge_threshold  = 22.1f;
    /// 放电时母线稳压目标
    static constexpr float k_vin_reg_target           = 22.2f;

    // ====== 充放电限制 ======
    /// 最大充电功率 (W)
    static constexpr float k_p_charge_max        = 190.0f;
    /// 恒输入功率模式下允许的最小功率目标 (W)
    static constexpr float k_bus_power_min       = 10.0f;
    /// 电容电流限制 (A)，|ccap| 不应超过此值
    static constexpr float k_vcap_current_limit  = 50.0f;
    /// 功率变化速率限制：10 kW/s，对应 20 kHz 控制环 0.5 W/tick
    static constexpr float k_power_ramp_rate     = 10000.0f;

    // ====== 软启动限制 ======
    /// 电容电压低于此值时软启动目标功率受限
    static constexpr float k_soft_start_vcap_threshold = k_vcap_min;
    /// 电容欠压时软启动允许的最大功率 (W)
    static constexpr float k_soft_start_vcap_power_limit = 35.0f;

    // ====== 稳压滞回与容差 ======
    /// 上限稳压退出滞回 (vcap < k_vcap_high - 此值 才退出)
    static constexpr float k_vcap_high_hysteresis = 0.5f;
    /// 下限稳压切放电滞回 (vcap > k_vcap_low + 此值 才允许放电)
    static constexpr float k_vcap_low_hysteresis  = 1.0f;
    /// vcap_reg_target_ 目标判断容差 (用于区分 high/low 目标)
    static constexpr float k_vcap_target_tolerance = 0.1f;
    
    static constexpr float k_vcap_high_tolerance = 0.1f;
    static constexpr float k_vcap_low_tolerance = 0.8f;



    // ====== 前馈变比限制 ======
    /// ffd_ratio 上限，留余量低于变换器最大变比 1.32
    static constexpr float k_ffd_max = 1.25f;

    // ====== 保护阈值 ======
    static constexpr float k_vin_low_threshold   = 15.0f;   ///< 母线欠压保护
    static constexpr float k_vin_high_threshold  = 28.0f;   ///< 母线过压保护
    static constexpr float k_vin_recover_low     = 16.0f;   ///< 母线恢复阈值 (下)
    static constexpr float k_vin_recover_high    = 27.0f;   ///< 母线恢复阈值 (上)

    /// CAN 通信超时 (ms)
    static constexpr uint32_t k_can_timeout_ms = 100;
    /// 启动充电超时 (ms)，CAN 恢复后超过此时间仍未充至目标电压则强制退出
    static constexpr uint32_t k_startup_charge_timeout = 10000;

    void init();
    void dispatch(TIM_HandleTypeDef *htim);

    // ====== 控制状态访问 ======
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
    bool startup_charging() const { return startup_charging_; }
    uint32_t work_tick() const { return work_tick_; }

    const AdcSampler& sampler() const { return sampler_; }
    const SuperCapComm& comm() const { return *comm_; }
    void debugSync();

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
    PIDController vloop_;          ///< 电容电压环 (稳压)
    PIDController ploop_;          ///< 功率环 (充电)
    PIDController bus_vloop_;      ///< 母线电压环 (放电时稳压 Vin)
    SuperCapComm *comm_ = nullptr;

    BufferCapState state_ = BufferCapState::kIdle;
    float bus_power_target_ = 0.0f;   ///< 电池功率目标 (来自 CAN)，恒输入功率模式使用
    float power_set_ = 0.0f;          ///< 软启动当前允许的最大功率
    float cap_v_max_ = k_vcap_high;   ///< 电容电压上限 (调试用)
    float volt_ratio_ = 0.0f;         ///< 当前变比系数
    float ploop_ref_ = 0.0f;          ///< 功率环输出
    float vloop_ref_ = 0.0f;          ///< 电压环输出
    float cloop_ref_ = 0.0f;          ///< 取小后的电流环输出
    float vcap_reg_target_ = k_vcap_high;  ///< 稳压目标值 (k_vcap_high 或 k_vcap_low)

    bool restart_flag_ = false;       ///< 保护关断后等待恢复
    bool startup_charging_ = false;   ///< 启动充电模式 (CAN 刚连上且 vcap < k_vcap_min)
    int can_disable_cnt_ = 0;
    bool can_disable_flag_ = false;
    uint16_t power_start_tick_ = 0;
    uint16_t can_div_cnt_ = 0;
    uint32_t work_tick_ = 0;          ///< CAN 正常工作时间 (ms)，CAN 断连时清零
    uint32_t work_tick_base_ = 0;     ///< work_tick_ 基准时间戳

    DebugInterface debugger_;
};

} // namespace supercap

#endif // CONTROL_TASK_HPP

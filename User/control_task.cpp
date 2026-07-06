/**
 * @file    control_task.cpp
 * @brief   超级电容主控制任务类实现
 *
 * 控制环路架构：
 *
 *  四开关 Buck-Boost 双向变换器，能量流向由 volt_ratio 与 ffd_ratio 的关系决定：
 *    volt_ratio > ffd_ratio → 充电 (母线 → 电容)
 *    volt_ratio < ffd_ratio → 放电 (电容 → 母线)
 *    volt_ratio = ffd_ratio → 平衡 (无净能量流动)
 *
 *  电容电流方向：ccap > 0 充电，ccap < 0 放电
 *  电池电流方向：cin  > 0 从电池取电 (负载功率 + 电容功率 > 0)
 */
#ifdef __cplusplus
extern "C" {
#endif
#include "math.h"
#include "main.h"
#include "tim.h"
#include "hrtim.h"

#ifdef __cplusplus
}
#endif

#include "control_task.hpp"
#include "global_instances.hpp"
#include "debug_vars.h"

namespace supercap {

void ControlTask::init()
{
    comm_ = &g_super_cap_comm;

    // 电容电压环：稳压至 vcap_reg_target_ (k_vcap_high 或 k_vcap_low)
    vloop_.init(0.06f, 0.00001f, 0.06f, 0.02f, -0.06f, -0.02f);
    // 功率环：跟踪 CAN 电池功率指令
    ploop_.init(0.0f, 0.000005f, 0.06f, 0.06f, -0.06f, -0.06f);
    // 母线电压环：放电时维持 Vin = k_vin_reg_target
    // PI 控制 (ki=0.00001)，避免积分负向饱和导致放电响应延迟
    bus_vloop_.init(0.12f, 0.00001f, 0.2f, 0.02f, 0.0f, 0.0f);

    state_ = BufferCapState::kIdle;

    comm_->init();
    sampler_.startDMA();

    HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_E | HRTIM_TIMERID_MASTER);
    HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim4);
}

void ControlTask::dispatch(TIM_HandleTypeDef *htim)
{
    if (htim == &htim2) {
        onTim2();
    } else if (htim == &htim3) {
        onTim3();
    } else if (htim == &htim4) {
        onTim4();
    }
}

/// @brief TIM2 主控制环 (20 kHz / 50 µs)
void ControlTask::onTim2()
{
    sampleAndFilter();
    softStart();
    computeControlLoop();
    updatePwm();
    debugSync();
    debugger_.update(sampler_, *this, *comm_);
    debugger_.send();
}

void ControlTask::sampleAndFilter()
{
    sampler_.update();
    sampler_.calibrate(k_board_id);
    sampler_.applyComplementaryFilter(power_start_tick_ < k_soft_start_time);
}

/// @brief 软启动：功率目标以 10 kW/s 速率斜坡上升，无阶跃跳变
void ControlTask::softStart()
{
    // 目标功率
    float target = k_p_charge_max;

    // 电容欠压时限制充电功率
    if (sampler_.vcap() < k_soft_start_vcap_threshold && target > k_soft_start_vcap_power_limit) {
        target = k_soft_start_vcap_power_limit;
    }

    // 速率限制：5 kW/s ÷ 20 kHz = 0.25 W/tick
    constexpr float k_ramp_per_tick = k_power_ramp_rate / 20000.0f;
    if (power_set_ < target) {
        power_set_ += k_ramp_per_tick;
        if (power_set_ > target) power_set_ = target;
    } else if (power_set_ > target) {
        power_set_ -= k_ramp_per_tick;
        if (power_set_ < target) power_set_ = target;
    }

    // 硬限幅
    if (power_set_ > k_p_charge_max) power_set_ = k_p_charge_max;
    if (power_set_ < k_bus_power_min) power_set_ = k_bus_power_min;

    // 软启动计时器 (用于互补滤波)
    if (power_start_tick_ < k_soft_start_time) {
        power_start_tick_++;
    }
}

/// @brief 主控制状态机
void ControlTask::computeControlLoop()
{
    float vin  = sampler_.vin();
    float vcap = sampler_.vcap();
    float ccap = sampler_.ccap();

    // ====== 前馈变比计算 (含限幅) ======
    float vin_clamped = (vin < 15.0f) ? 15.0f : vin;
    float ffd_ratio_raw = vcap / vin_clamped;
    // 限制前馈比不超过变换器最大变比，留余量
    float ffd_ratio = (ffd_ratio_raw > k_ffd_max) ? k_ffd_max : ffd_ratio_raw;
    g_dbg_ffd_ratio = ffd_ratio;

    // ====== 启动充电模式退出 ======
    // 充至 vcap > k_vcap_low + k_vcap_low_hysteresis 后退出启动模式
    // 或 CAN 恢复后超过 k_startup_charge_timeout 仍未充至目标电压，强制退出
    if (startup_charging_ && (vcap > k_vcap_low + k_vcap_low_hysteresis || work_tick_ > k_startup_charge_timeout)) {
        startup_charging_ = false;
    }

    // ====== 全局硬关断保护 ======
    // vcap >= k_vcap_max：过压关断
    // vcap <= k_vcap_min 且非启动模式：欠压关断
    if (vcap >= k_vcap_max || (vcap <= k_vcap_min && !startup_charging_)) {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        state_ = BufferCapState::kIdle;
        vloop_.reset();
        ploop_.reset();
        bus_vloop_.reset();
        restart_flag_ = true;
        volt_ratio_ = ffd_ratio;
        return;
    }

    // ====== 全局稳压切换 ======
    // 过充保护：任何状态下 vcap 达到 k_vcap_high → 切稳压至 k_vcap_high
    if (vcap >= k_vcap_high && state_ != BufferCapState::kVoltageReg) {
        state_ = BufferCapState::kVoltageReg;
        vcap_reg_target_ = k_vcap_high;
        vloop_.reset();
    }
    // 过放保护：任何状态下 vcap 降至 k_vcap_low → 切稳压至 k_vcap_low
    else if (vcap <= k_vcap_low && state_ != BufferCapState::kVoltageReg && !startup_charging_) {
        state_ = BufferCapState::kVoltageReg;
        vcap_reg_target_ = k_vcap_low;
        vloop_.reset();
    }

    // ====== 状态机 ======
    float cloop_ratio;
    float vloop_ratio;

    switch (state_) {
    // -------------------- kIdle：平衡态 --------------------
    case BufferCapState::kIdle:
        volt_ratio_ = ffd_ratio;  // 平衡态，不主动充放电

        // 启动充电模式：CAN 已连接且电容严重欠压
        if (startup_charging_ && !can_disable_flag_) {
            state_ = BufferCapState::kPower;
            ploop_.reset();
            vloop_.reset();
        }
        // 恒输入功率模式：母线电压足够高且电容未满
        else if (vin > k_vin_charge_threshold && vcap < k_vcap_high && !can_disable_flag_) {
            state_ = BufferCapState::kPower;
            ploop_.reset();
            vloop_.reset();
        }
        // 主动放电：母线电压过低且电容有能量
        else if (vin < k_vin_discharge_threshold && vcap > k_vcap_low && !can_disable_flag_) {
            state_ = BufferCapState::kDischarging;
            bus_vloop_.reset();
        }

        // 过充保护：任何状态下 vcap 达到 k_vcap_high → 切稳压至 k_vcap_high
        if (vcap >= k_vcap_high) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_high;
            vloop_.reset();
        }
        // 过放保护：任何状态下 vcap 降至 k_vcap_low → 切稳压至 k_vcap_low
        else if (vcap <= k_vcap_low && !startup_charging_) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_low;
            vloop_.reset();
        }

        break;

    // -------------------- kPower：恒输入功率模式 --------------------
    case BufferCapState::kPower:
        // 母线电压过低 → 切换放电
        if (vin < k_vin_discharge_threshold && vcap > k_vcap_low) {
            state_ = BufferCapState::kDischarging;
            bus_vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // vcap 达到上限 → 切稳压
        if (vcap >= k_vcap_high - k_vcap_high_tolerance) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_high;
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // vcap 降至下限 → 切稳压
        if (vcap <= k_vcap_low + k_vcap_low_tolerance && !startup_charging_) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_low;
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }

        // 电池功率目标来自 CAN 指令
        bus_power_target_ = comm_->chassis2cap_msg.battery_power;
        // 软启动功率限制
        if (bus_power_target_ > power_set_) bus_power_target_ = power_set_;
        if (bus_power_target_ > k_p_charge_max) bus_power_target_ = k_p_charge_max;
        if (bus_power_target_ < k_bus_power_min) bus_power_target_ = k_bus_power_min;

        //TODO: 测试
        bus_power_target_ = 60.0f; // 测试用

        // 功率环 + 电压环取小
        ploop_ref_ = ploop_.compute(sampler_.pin(), bus_power_target_);
        vloop_ref_ = vloop_.compute(vcap, k_vcap_high);
        cloop_ref_ = ploop_ref_;

        cloop_ratio = ffd_ratio + cloop_ref_;
        vloop_ratio = ffd_ratio + vloop_ref_;
        volt_ratio_ = (vloop_ratio <= cloop_ratio) ? vloop_ratio : cloop_ratio;

        // 过充保护：任何状态下 vcap 达到 k_vcap_high → 切稳压至 k_vcap_high
        if (vcap >= k_vcap_high) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_high;
            vloop_.reset();
        }
        // 过放保护：任何状态下 vcap 降至 k_vcap_low → 切稳压至 k_vcap_low
        else if (vcap <= k_vcap_low && !startup_charging_) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_low;
            vloop_.reset();
        }

        break;

    // -------------------- kVoltageReg：电压环稳压 --------------------
    case BufferCapState::kVoltageReg:
        // 母线电压过低且电容有能量 → 切换放电 (带滞回，避免边界振荡)
        if (vin < k_vin_discharge_threshold && vcap > k_vcap_low + k_vcap_low_hysteresis) {
            state_ = BufferCapState::kDischarging;
            bus_vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // 稳压上限模式：vcap 回落 → 重新恒功率
        if (fabsf(vcap_reg_target_ - k_vcap_high) < k_vcap_target_tolerance && vcap < k_vcap_high - k_vcap_high_hysteresis) {
            state_ = BufferCapState::kPower;
            ploop_.reset();
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // 稳压下限模式：母线恢复 → 重新恒功率
        if (fabsf(vcap_reg_target_ - k_vcap_low) < k_vcap_target_tolerance && vin > k_vin_charge_threshold) {
            state_ = BufferCapState::kPower;
            ploop_.reset();
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }

        // 电压环稳压
        vloop_ref_ = vloop_.compute(vcap, vcap_reg_target_);

        // 稳压下限模式：禁止放电 (vcap_ref < 0 时钳零)，仅允许充电维持
        if (fabsf(vcap_reg_target_ - k_vcap_low) < k_vcap_target_tolerance && vloop_ref_ < 0.0f) {
            vloop_ref_ = 0.0f;
        }

        volt_ratio_ = ffd_ratio + vloop_ref_;
        break;

    // -------------------- kDischarging：母线电压环放电 --------------------
    case BufferCapState::kDischarging:
        // vcap 降至下限 → 切稳压 (全局检查已处理，此处为安全冗余)
        if (vcap <= k_vcap_low) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_low;
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // 母线电压恢复 → 重新恒功率
        if (vin > k_vin_charge_threshold) {
            state_ = BufferCapState::kPower;
            ploop_.reset();
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // vcap 达到上限 (异常情况) → 切稳压
        if (vcap >= k_vcap_high) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_high;
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }

        // 母线电压环：放电维持 Vin = k_vin_reg_target
        // error = k_vin_reg_target - vin，vin 低于目标时 error 为正
        // 放电对应 volt_ratio < ffd_ratio，故 volt_ratio_ = ffd_ratio - bus_vloop_.out
        // bus_vloop_.out ∈ [0, 0.2] (out_min=0)，保证该模式绝不充电
        bus_vloop_.compute(vin, k_vin_reg_target);
        volt_ratio_ = ffd_ratio - bus_vloop_.out;

        // 过充保护：任何状态下 vcap 达到 k_vcap_high → 切稳压至 k_vcap_high
        if (vcap >= k_vcap_high) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_high;
            vloop_.reset();
        }
        // 过放保护：任何状态下 vcap 降至 k_vcap_low → 切稳压至 k_vcap_low
        else if (vcap <= k_vcap_low && !startup_charging_) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_low;
            vloop_.reset();
        }

        break;
    }

    // ====== 电容电流限制 (动态钳位) ======
    // 当 |ccap| 接近 k_vcap_current_limit 时，线性收紧 volt_ratio 偏离 ffd_ratio 的余量
    // 80% 限值以下：全余量 0.2；80%~100%：线性降至 0；超限：强制拉回 ffd_ratio
    float abs_ccap = (ccap < 0.0f) ? -ccap : ccap;
    float margin = 0.2f;
    if (abs_ccap > k_vcap_current_limit * 0.8f) {
        margin = 0.2f * (k_vcap_current_limit - abs_ccap) / (k_vcap_current_limit * 0.2f);
        if (margin < 0.0f) margin = 0.0f;
    }
    // 充电方向 (ccap > 0)：收紧上界
    float charge_margin    = (ccap > 0.0f) ? margin : 0.2f;
    // 放电方向 (ccap < 0)：收紧下界
    float discharge_margin = (ccap < 0.0f) ? margin : 0.2f;

    float upper_bound = ffd_ratio + charge_margin;
    float lower_bound = ffd_ratio - discharge_margin;
    if (volt_ratio_ > upper_bound) volt_ratio_ = upper_bound;
    if (volt_ratio_ < lower_bound) volt_ratio_ = lower_bound;

    // ====== 硬编码安全钳位 (固定值，最终安全网) ======
    float ff_bound = ffd_ratio;
    if (volt_ratio_ > ff_bound + 0.2f) volt_ratio_ = ff_bound + 0.2f;
    if (volt_ratio_ < ff_bound - 0.2f) volt_ratio_ = ff_bound - 0.2f;
}

void ControlTask::updatePwm()
{
    converter_.update(volt_ratio_);
}

/// @brief TIM3 保护检查 (200 kHz / 5 µs)
void ControlTask::onTim3()
{
    checkProtection();
}

void ControlTask::checkProtection()
{
    // ====== CAN 通信超时检测 ======
    // 单次超时即判定 CAN 断连
    can_disable_flag_ = (HAL_GetTick() - comm_->last_tick() > k_can_timeout_ms);
    can_disable_cnt_ = can_disable_flag_ ? 1 : 0;

    // ====== work_tick_ 管理：记录 CAN 正常接收后的持续时间 (ms) ======
    if (can_disable_flag_) {
        work_tick_ = 0;
        work_tick_base_ = 0;
    } else {
        if (work_tick_base_ == 0) {
            work_tick_base_ = HAL_GetTick();
        }
        work_tick_ = HAL_GetTick() - work_tick_base_;
    }

    // ====== 异常检测：CAN 断连、母线过压/欠压、电容过压、电容过流 ======
    bool abnormal = can_disable_flag_ ||
                    (sampler_.vin() < k_vin_low_threshold) ||
                    (sampler_.vin() > k_vin_high_threshold) ||
                    (sampler_.vcap() > k_vcap_max) ||
                    (fabsf(sampler_.ccap()) > k_vcap_current_limit * 1.5f);

    if (abnormal) {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        state_ = BufferCapState::kIdle;
        vloop_.reset();
        ploop_.reset();
        bus_vloop_.reset();
        restart_flag_ = true;
    }

    // ====== 恢复检测 ======
    bool recovered = !can_disable_flag_ &&
                     (sampler_.vin() > k_vin_recover_low) &&
                     (sampler_.vin() < k_vin_recover_high) &&
                     (sampler_.vcap() < k_vcap_max) &&
                    ((sampler_.vcap() > k_vcap_min + k_vcap_low_tolerance && work_tick_ > k_startup_charge_timeout) || 
                     (work_tick_ <= k_startup_charge_timeout)) &&
                     restart_flag_;

    if (recovered) {
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        restart_flag_ = false;
        // 恢复时复位 PID，避免历史积分造成冲击
        vloop_.reset();
        ploop_.reset();
        bus_vloop_.reset();
        // 启动充电模式：CAN 刚恢复且电容严重欠压时，豁免 k_vcap_min 关断
        if (sampler_.vcap() < k_vcap_min) {
            startup_charging_ = true;
        }
    }
}

/// @brief TIM4 CAN 通信发送 (10 kHz / 100 µs, 19 分频 ≈ 526 Hz)
void ControlTask::onTim4()
{
    can_div_cnt_++;
    if (can_div_cnt_ == 19) {
        // 更新发送消息
        comm_->cap2chassis_msg.vin  = sampler_.vin();
        comm_->cap2chassis_msg.vcap = sampler_.vcap();
        comm_->cap2chassis_msg.ccap = sampler_.ccap();
        comm_->cap2chassis_msg.cin  = sampler_.cin();

        // 剩余电量百分比：按储能 E = ½CV² 计算，以 k_vcap_low 为 0%，k_vcap_high 为 100%
        float vcap_sq = sampler_.vcap() * sampler_.vcap();
        float low_sq  = k_vcap_low * k_vcap_low;
        float high_sq = k_vcap_high * k_vcap_high;
        float remain = (vcap_sq - low_sq) / (high_sq - low_sq);
        if (remain > 1.0f) remain = 1.0f;
        if (remain < 0.0f) remain = 0.0f;
        comm_->cap2chassis_msg.remain_pct = remain * 100.0f;

        can_div_cnt_ = 0;
        comm_->send();
    }
}

void ControlTask::debugSync()
{
    // ========== AdcSampler ==========
    const Sample_struct_t *s = sampler_.sample_buf();
    g_dbg_vin         = sampler_.vin();
    g_dbg_cin         = sampler_.cin();
    g_dbg_cout        = sampler_.cout();
    g_dbg_vcap        = sampler_.vcap();
    g_dbg_ccap        = sampler_.ccap();
    g_dbg_vout        = sampler_.vout();
    g_dbg_vin_last    = sampler_.vin_last();
    g_dbg_vcap_last   = sampler_.vcap_last();

    for (int i = 0; i < AdcSampler::k_buf_len; i++) {
        g_dbg_raw_vin[i]  = s->vin[i];
        g_dbg_raw_cin[i]  = s->cin[i];
        g_dbg_raw_cout[i] = s->cout[i];
        g_dbg_raw_vcap[i] = s->vcap[i];
        g_dbg_raw_ccap[i] = s->ccap[i];
    }

    // ========== ControlTask ==========
    g_dbg_state            = static_cast<uint8_t>(state_);
    g_dbg_bus_power_target = bus_power_target_;
    g_dbg_power_set        = power_set_;
    g_dbg_cap_v_max        = cap_v_max_;
    g_dbg_volt_ratio       = volt_ratio_;
    g_dbg_ploop_ref        = ploop_ref_;
    g_dbg_vloop_ref        = vloop_ref_;
    g_dbg_cloop_ref        = cloop_ref_;
    g_dbg_restart_flag     = restart_flag_;
    g_dbg_can_disable_cnt  = can_disable_cnt_;
    g_dbg_can_disable_flag = can_disable_flag_;
    g_dbg_power_start_tick = power_start_tick_;
    g_dbg_can_div_cnt      = can_div_cnt_;
    g_dbg_startup_charging = startup_charging_;
    g_dbg_vcap_reg_target  = vcap_reg_target_;
    g_dbg_work_tick        = work_tick_;

    // ========== PID 控制器 ==========
    g_dbg_vloop_kp       = vloop_.kp;
    g_dbg_vloop_ki       = vloop_.ki;
    g_dbg_vloop_pout     = vloop_.pout;
    g_dbg_vloop_iout     = vloop_.iout;
    g_dbg_vloop_out      = vloop_.out;
    g_dbg_vloop_err[0]   = vloop_.error[0];
    g_dbg_vloop_err[1]   = vloop_.error[1];

    g_dbg_ploop_kp       = ploop_.kp;
    g_dbg_ploop_ki       = ploop_.ki;
    g_dbg_ploop_pout     = ploop_.pout;
    g_dbg_ploop_iout     = ploop_.iout;
    g_dbg_ploop_out      = ploop_.out;
    g_dbg_ploop_err[0]   = ploop_.error[0];
    g_dbg_ploop_err[1]   = ploop_.error[1];

    g_dbg_busvloop_kp       = bus_vloop_.kp;
    g_dbg_busvloop_ki       = bus_vloop_.ki;
    g_dbg_busvloop_pout     = bus_vloop_.pout;
    g_dbg_busvloop_iout     = bus_vloop_.iout;
    g_dbg_busvloop_out      = bus_vloop_.out;
    g_dbg_busvloop_err[0]   = bus_vloop_.error[0];
    g_dbg_busvloop_err[1]   = bus_vloop_.error[1];

    // ========== Buck-Boost ==========
    g_dbg_buck_duty  = converter_.buck_duty();
    g_dbg_boost_duty = converter_.boost_duty();

    // ========== CAN 通信 ==========
    g_dbg_battery_power = comm_->chassis2cap_msg.battery_power;
    g_dbg_can_last_tick = comm_->last_tick();
}

} // namespace supercap

// C链接的 HAL 定时器回调桥接
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    supercap::g_control_task.dispatch(htim);
}

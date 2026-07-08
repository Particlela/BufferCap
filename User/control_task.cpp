/**
 * @file    control_task.cpp
 * @brief   Super capacitor control task implementation.
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

    cap_high_vloop_.init(0.06f, 0.00001f, 0.06f, 0.02f, -0.06f, -0.02f);
    cap_low_vloop_.init(0.06f, 0.00001f, 0.06f, 0.02f, -0.06f, -0.02f);
    ploop_.init(0.0f, 0.000005f, 0.06f, 0.06f, -0.06f, -0.06f);
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

void ControlTask::onTim2()
{
    sampleAndFilter();
    softStart();
    computeControlLoop();
    updatePwm();
    debugSync();
    debugger_.update(sampler_, *this, *comm_);

    debug_div_cnt_++;
    if (debug_div_cnt_ >= k_debug_send_div) {
        debug_div_cnt_ = 0;
        debugger_.send();
    }
}

void ControlTask::sampleAndFilter()
{
    sampler_.update();
    sampler_.calibrate(k_board_id);
    sampler_.applyComplementaryFilter(power_start_tick_ < k_soft_start_time);
}

void ControlTask::softStart()
{
    float target = k_p_charge_max;

    if (sampler_.vcap() < k_soft_start_vcap_threshold && target > k_soft_start_vcap_power_limit) {
        target = k_soft_start_vcap_power_limit;
    }

    constexpr float k_ramp_per_tick = k_power_ramp_rate / 20000.0f;
    if (power_set_ < target) {
        power_set_ += k_ramp_per_tick;
        if (power_set_ > target) power_set_ = target;
    } else if (power_set_ > target) {
        power_set_ -= k_ramp_per_tick;
        if (power_set_ < target) power_set_ = target;
    }

    if (power_set_ > k_p_charge_max) power_set_ = k_p_charge_max;
    if (power_set_ < k_bus_power_min) power_set_ = k_bus_power_min;

    if (power_start_tick_ < k_soft_start_time) {
        power_start_tick_++;
    }
}

void ControlTask::resetVoltageLoops()
{
    cap_high_vloop_.reset();
    cap_low_vloop_.reset();
}

void ControlTask::computeControlLoop()
{
    float vin  = sampler_.vin();
    float vcap = sampler_.vcap();
    float ccap = sampler_.ccap();
    float cout = sampler_.cout();

    float vin_clamped = (vin < 15.0f) ? 15.0f : vin;
    float ffd_ratio_raw = vcap / vin_clamped;
    float ffd_ratio = (ffd_ratio_raw > k_ffd_max) ? k_ffd_max : ffd_ratio_raw;
    g_dbg_ffd_ratio = ffd_ratio;

    pcap_ = vcap * ccap;
    load_power_est_ = sampler_.pin() - pcap_;
    can_charge_cap_ = !can_disable_flag_ &&
                      (vin > k_vin_charge_threshold) &&
                      (comm_->chassis2cap_msg.battery_power > load_power_est_ + k_charge_power_margin);

    bus_power_target_ = comm_->chassis2cap_msg.battery_power;
    if (bus_power_target_ > power_set_) bus_power_target_ = power_set_;
    if (bus_power_target_ > k_p_charge_max) bus_power_target_ = k_p_charge_max;
    if (bus_power_target_ < k_bus_power_min) bus_power_target_ = k_bus_power_min;

    if (low_clamp_active_ && (vcap >= k_vcap_clamp_exit || can_charge_cap_)) {
        low_clamp_active_ = false;
    }
    if (!low_clamp_active_ && vcap <= k_vcap_min && !can_charge_cap_) {
        low_clamp_active_ = true;
    }

    if (vcap >= k_vcap_max) {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        state_ = BufferCapState::kIdle;
        resetVoltageLoops();
        ploop_.reset();
        bus_vloop_.reset();
        restart_flag_ = true;
        low_clamp_active_ = false;
        volt_ratio_ = ffd_ratio;
        return;
    }

    if (vcap >= k_vcap_high && state_ != BufferCapState::kVoltageReg) {
        state_ = BufferCapState::kVoltageReg;
        vcap_reg_target_ = k_vcap_high;
        cap_high_vloop_.reset();
    }

    float cloop_ratio = ffd_ratio;
    float vloop_ratio = ffd_ratio;
    bool handled = false;

    if (low_clamp_active_ && !can_charge_cap_) {
        if (state_ != BufferCapState::kVoltageReg ||
            fabsf(vcap_reg_target_ - k_vcap_low) >= k_vcap_target_tolerance) {
            state_ = BufferCapState::kVoltageReg;
            vcap_reg_target_ = k_vcap_low;
            cap_low_vloop_.reset();
        }
        ploop_.reset();
        bus_vloop_.reset();
        ploop_ref_ = 0.0f;
        cloop_ref_ = 0.0f;
        vloop_ref_ = (k_vcap_min / vin_clamped) - ffd_ratio;
        volt_ratio_ = k_vcap_min / vin_clamped;
        handled = true;
    }

    if (!handled) {
        switch (state_) {
        case BufferCapState::kIdle:
            volt_ratio_ = ffd_ratio;

            if (vcap <= k_vcap_low && !can_charge_cap_) {
                state_ = BufferCapState::kVoltageReg;
                vcap_reg_target_ = k_vcap_low;
                cap_low_vloop_.reset();
            } else if (vin > k_vin_charge_threshold && vcap < k_vcap_high && !can_disable_flag_) {
                state_ = BufferCapState::kPower;
                ploop_.reset();
                resetVoltageLoops();
            } else if (vin < k_vin_discharge_threshold && vcap > k_vcap_low && !can_disable_flag_ && cout > k_cout_discharge_threshold) {
                state_ = BufferCapState::kDischarging;
                bus_vloop_.reset();
            }
            break;

        case BufferCapState::kPower:
            if (vcap >= k_vcap_high - k_vcap_high_tolerance) {
                state_ = BufferCapState::kVoltageReg;
                vcap_reg_target_ = k_vcap_high;
                cap_high_vloop_.reset();
                volt_ratio_ = ffd_ratio;
                break;
            }
            if (vcap <= k_vcap_low && !can_charge_cap_) {
                state_ = BufferCapState::kVoltageReg;
                vcap_reg_target_ = k_vcap_low;
                cap_low_vloop_.reset();
                volt_ratio_ = ffd_ratio;
                break;
            }
            if (vin < k_vin_discharge_threshold && vcap > k_vcap_low && cout > k_cout_discharge_threshold) {
                state_ = BufferCapState::kDischarging;
                bus_vloop_.reset();
                volt_ratio_ = ffd_ratio;
                break;
            }

            ploop_ref_ = ploop_.compute(sampler_.pin(), bus_power_target_);
            if (vcap <= k_vcap_low && can_charge_cap_ && ploop_ref_ < 0.0f) {
                ploop_.reset();
                ploop_ref_ = ploop_.compute(sampler_.pin(), bus_power_target_);
                if (ploop_ref_ < 0.0f) ploop_ref_ = 0.0f;
            }
            vloop_ref_ = cap_high_vloop_.compute(vcap, k_vcap_high);
            cloop_ref_ = ploop_ref_;

            cloop_ratio = ffd_ratio + cloop_ref_;
            vloop_ratio = ffd_ratio + vloop_ref_;
            volt_ratio_ = (vloop_ratio <= cloop_ratio) ? vloop_ratio : cloop_ratio;
            break;

        case BufferCapState::kVoltageReg:
            if (fabsf(vcap_reg_target_ - k_vcap_high) < k_vcap_target_tolerance) {
                if (vin < k_vin_discharge_threshold && vcap > k_vcap_low + k_vcap_low_hysteresis && cout > k_cout_discharge_threshold) {
                    state_ = BufferCapState::kDischarging;
                    bus_vloop_.reset();
                    volt_ratio_ = ffd_ratio;
                    break;
                }
                // if (vcap < k_vcap_high - k_vcap_high_hysteresis) {
                //     state_ = BufferCapState::kPower;
                //     ploop_.reset();
                //     resetVoltageLoops();
                //     volt_ratio_ = ffd_ratio;
                //     break;
                // }

                vloop_ref_ = cap_high_vloop_.compute(vcap, k_vcap_high);
                volt_ratio_ = ffd_ratio + vloop_ref_;
            } else {
                if (can_charge_cap_) {
                    state_ = BufferCapState::kPower;
                    ploop_.reset();
                    resetVoltageLoops();
                    volt_ratio_ = ffd_ratio;
                    break;
                }

                vloop_ref_ = cap_low_vloop_.compute(vcap, k_vcap_low);
                volt_ratio_ = ffd_ratio + vloop_ref_;
            }
            break;

        case BufferCapState::kDischarging:
            if (vcap <= k_vcap_low) {
                if (can_charge_cap_) {
                    state_ = BufferCapState::kPower;
                    ploop_.reset();
                    resetVoltageLoops();
                } else {
                    state_ = BufferCapState::kVoltageReg;
                    vcap_reg_target_ = k_vcap_low;
                    cap_low_vloop_.reset();
                }
                volt_ratio_ = ffd_ratio;
                break;
            }
            if (vin > k_vin_charge_threshold || cout < k_cout_discharge_threshold * 0.8f) {
                state_ = BufferCapState::kPower;
                ploop_.reset();
                resetVoltageLoops();
                volt_ratio_ = ffd_ratio;
                break;
            }
            if (vcap >= k_vcap_high) {
                state_ = BufferCapState::kVoltageReg;
                vcap_reg_target_ = k_vcap_high;
                cap_high_vloop_.reset();
                volt_ratio_ = ffd_ratio;
                break;
            }

            bus_vloop_.compute(vin, k_vin_reg_target);
            volt_ratio_ = ffd_ratio - bus_vloop_.out;
            break;
        }
    }

    float abs_ccap = (ccap < 0.0f) ? -ccap : ccap;
    float margin = 0.2f;
    if (abs_ccap > k_vcap_current_limit * 0.95f) {
        margin = 0.2f * (k_vcap_current_limit - abs_ccap) / (k_vcap_current_limit * 0.05f);
        if (margin < 0.0f) margin = 0.0f;
    }

    float charge_margin    = (ccap > 0.0f) ? margin : 0.2f;
    float discharge_margin = (ccap < 0.0f) ? margin : 0.2f;

    float upper_bound = ffd_ratio + charge_margin;
    float lower_bound = ffd_ratio - discharge_margin;
    if (volt_ratio_ > upper_bound) volt_ratio_ = upper_bound;
    if (volt_ratio_ < lower_bound) volt_ratio_ = lower_bound;

    if (volt_ratio_ > ffd_ratio + 0.2f) volt_ratio_ = ffd_ratio + 0.2f;
    if (volt_ratio_ < ffd_ratio - 0.2f) volt_ratio_ = ffd_ratio - 0.2f;
}

void ControlTask::updatePwm()
{
    converter_.update(volt_ratio_);
}

void ControlTask::onTim3()
{
    checkProtection();
}

void ControlTask::checkProtection()
{
    can_disable_flag_ = (HAL_GetTick() - comm_->last_tick() > k_can_timeout_ms);
    can_disable_cnt_ = can_disable_flag_ ? 1 : 0;

    if (can_disable_flag_) {
        work_tick_ = 0;
        work_tick_base_ = 0;
    } else {
        if (work_tick_base_ == 0) {
            work_tick_base_ = HAL_GetTick();
        }
        work_tick_ = HAL_GetTick() - work_tick_base_;
    }

    bool abnormal = can_disable_flag_ ||
                    (sampler_.vin() < k_vin_low_threshold) ||
                    (sampler_.vin() > k_vin_high_threshold) ||
                    (sampler_.vcap() > k_vcap_max) ||
                    (fabsf(sampler_.ccap()) > k_vcap_current_limit * 1.5f);

    if (abnormal) {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        state_ = BufferCapState::kIdle;
        resetVoltageLoops();
        ploop_.reset();
        bus_vloop_.reset();
        restart_flag_ = true;
        low_clamp_active_ = false;
    }

    bool recovered = !can_disable_flag_ &&
                     (sampler_.vin() > k_vin_recover_low) &&
                     (sampler_.vin() < k_vin_recover_high) &&
                     (sampler_.vcap() < k_vcap_max) &&
                     restart_flag_;

    if (recovered) {
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        restart_flag_ = false;
        resetVoltageLoops();
        ploop_.reset();
        bus_vloop_.reset();
    }
}

void ControlTask::onTim4()
{
    can_div_cnt_++;
    if (can_div_cnt_ == 19) {
        comm_->cap2chassis_msg.vin  = sampler_.vin();
        comm_->cap2chassis_msg.vcap = sampler_.vcap();
        comm_->cap2chassis_msg.ccap = sampler_.ccap();
        comm_->cap2chassis_msg.cin  = sampler_.cin();

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

    g_dbg_state            = static_cast<uint8_t>(state_);
    g_dbg_chassis_power_target = comm_->chassis2cap_msg.battery_power;
    g_dbg_bus_power_target = bus_power_target_;
    g_dbg_power_set        = power_set_;
    g_dbg_cap_v_max        = cap_v_max_;
    g_dbg_volt_ratio       = volt_ratio_;
    g_dbg_ploop_ref        = ploop_ref_;
    g_dbg_vloop_ref        = vloop_ref_;
    g_dbg_cloop_ref        = cloop_ref_;
    g_dbg_bus_vloop_out    = bus_vloop_.out;
    g_dbg_restart_flag     = restart_flag_;
    g_dbg_can_disable_cnt  = can_disable_cnt_;
    g_dbg_can_disable_flag = can_disable_flag_;
    g_dbg_power_start_tick = power_start_tick_;
    g_dbg_can_div_cnt      = can_div_cnt_;
    g_dbg_low_clamp_active = low_clamp_active_;
    g_dbg_can_charge_cap   = can_charge_cap_;
    g_dbg_pcap             = pcap_;
    g_dbg_load_power_est   = load_power_est_;
    g_dbg_vcap_reg_target  = vcap_reg_target_;
    g_dbg_work_tick        = work_tick_;

    const PIDController *active_vloop = &cap_high_vloop_;
    if (state_ == BufferCapState::kVoltageReg &&
        fabsf(vcap_reg_target_ - k_vcap_low) < k_vcap_target_tolerance) {
        active_vloop = &cap_low_vloop_;
    }

    g_dbg_vloop_kp       = active_vloop->kp;
    g_dbg_vloop_ki       = active_vloop->ki;
    g_dbg_vloop_pout     = active_vloop->pout;
    g_dbg_vloop_iout     = active_vloop->iout;
    g_dbg_vloop_out      = active_vloop->out;
    g_dbg_vloop_err[0]   = active_vloop->error[0];
    g_dbg_vloop_err[1]   = active_vloop->error[1];

    g_dbg_high_vloop_kp       = cap_high_vloop_.kp;
    g_dbg_high_vloop_ki       = cap_high_vloop_.ki;
    g_dbg_high_vloop_pout     = cap_high_vloop_.pout;
    g_dbg_high_vloop_iout     = cap_high_vloop_.iout;
    g_dbg_high_vloop_out      = cap_high_vloop_.out;
    g_dbg_high_vloop_err[0]   = cap_high_vloop_.error[0];
    g_dbg_high_vloop_err[1]   = cap_high_vloop_.error[1];

    g_dbg_low_vloop_kp       = cap_low_vloop_.kp;
    g_dbg_low_vloop_ki       = cap_low_vloop_.ki;
    g_dbg_low_vloop_pout     = cap_low_vloop_.pout;
    g_dbg_low_vloop_iout     = cap_low_vloop_.iout;
    g_dbg_low_vloop_out      = cap_low_vloop_.out;
    g_dbg_low_vloop_err[0]   = cap_low_vloop_.error[0];
    g_dbg_low_vloop_err[1]   = cap_low_vloop_.error[1];

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

    g_dbg_buck_duty  = converter_.buck_duty();
    g_dbg_boost_duty = converter_.boost_duty();

    g_dbg_battery_power = comm_->chassis2cap_msg.battery_power;
    g_dbg_can_last_tick = comm_->last_tick();
}

} // namespace supercap

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    supercap::g_control_task.dispatch(htim);
}

/**
 * @file    control_task.cpp
 * @brief   超级电容主控制任务类实现
 */
#ifdef __cplusplus
extern "C" {
#endif

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

    // 电容电压环：维持 vcap_target=28.5V
    vloop_.init(0.06f, 0.00001f, 0.06f, 0.02f, -0.06f, -0.02f);
    // 功率环：跟踪 CAN 底盘功率指令
    ploop_.init(0.0f, 0.000005f, 0.06f, 0.06f, -0.06f, -0.06f);
    // 母线电压环：放电时维持 Vin=22.5V
    // error = 22.5V - Vin，输出加到 ffd_ratio 上调节能量流动
    bus_vloop_.init(0.3f, 0.00001f, 0.2f, 0.02f, 0.0f, -0.02f);

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
    debugger_.send();
}

void ControlTask::sampleAndFilter()
{
    sampler_.update();
    sampler_.calibrate(k_board_id);
    sampler_.applyComplementaryFilter(power_start_tick_ < k_soft_start_time);
}

void ControlTask::softStart()
{
    // 缓冲电容模式：软启动期间功率目标从 0 缓慢上升到 k_p_charge_max
    if (power_start_tick_ < k_soft_start_time) {
        if (power_set_ < k_p_charge_max) {
            power_set_ += 0.01f;
        } else {
            power_set_ = k_p_charge_max;
        }
        power_start_tick_++;
    } else {
        power_set_ = k_p_charge_max;
    }

    if (sampler_.vcap() < 10.0f && power_set_ > 35.0f) {
        power_set_ = 35.0f;
    }

    if (power_set_ > k_p_charge_max) power_set_ = k_p_charge_max;
    if (power_set_ < 10.0f) power_set_ = 10.0f;
}

void ControlTask::computeControlLoop()
{
    float vin = sampler_.vin();
    float vcap = sampler_.vcap();
    float vin_clamped = (vin < 15.0f) ? 15.0f : vin;
    float ffd_ratio = vcap / vin_clamped;
    float cloop_ratio;
    float vloop_ratio;

    switch (state_) {
    case BufferCapState::kIdle:
        //TODO: 如果是电容欠压导致的切入闲置，不应该使其等于ffd_ratio,应该使其等于 k_vcap_low / vin_clamped
        volt_ratio_ = ffd_ratio;
        // 母线电压高于充电阈值、电容未满、CAN 有效 → 开始充电
        if (vin > k_vin_charge_threshold && vcap < k_vcap_max && !can_disable_flag_) {
            state_ = BufferCapState::kCharging;
            ploop_.reset();
            vloop_.reset();
        }
        // 母线电压低于放电阈值且电容有余量 → 主动放电
        else if (vin < k_vin_discharge_threshold && vcap > k_vcap_danger_low && !can_disable_flag_) {
            state_ = BufferCapState::kDischarging;
            bus_vloop_.reset();
        }
        break;

    case BufferCapState::kCharging:
        // 母线电压低于放电阈值 → 切换放电
        if (vin < k_vin_discharge_threshold) {
            state_ = BufferCapState::kDischarging;
            bus_vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // 电容电压达到目标 → 切换稳压
        if (vcap >= k_vcap_target) {
            state_ = BufferCapState::kVoltageReg;
            vloop_.reset();
        }

        // 充电功率目标来自 CAN 底盘指令
        bus_power_target_ = comm_->chassis2cap_msg.chassis_power;
        // TODO:仅测试
        bus_power_target_ = 50.0f;
        if (bus_power_target_ > power_set_) bus_power_target_ = power_set_;  // 软启动限制
        if (bus_power_target_ > k_p_charge_max) bus_power_target_ = k_p_charge_max;
        if (bus_power_target_ < 10.0f) bus_power_target_ = 10.0f;

        ploop_ref_ = ploop_.compute(sampler_.pin(), bus_power_target_);
        vloop_ref_ = vloop_.compute(vcap, k_vcap_target);
        cloop_ref_ = ploop_ref_;

        cloop_ratio = ffd_ratio + cloop_ref_;
        vloop_ratio = ffd_ratio + vloop_ref_;
        volt_ratio_ = (vloop_ratio <= cloop_ratio) ? vloop_ratio : cloop_ratio;
        g_dbg_ffd_ratio = ffd_ratio;
        
        break;

    case BufferCapState::kVoltageReg:
        // 母线电压过低 → 主动放电支撑
        if (vin < k_vin_discharge_threshold) {
            state_ = BufferCapState::kDischarging;
            bus_vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }
        // 电容电压回落 → 重新充电
        if (vcap < k_vcap_target - 1.0f) {
            state_ = BufferCapState::kCharging;
            ploop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }

        vloop_ref_ = vloop_.compute(vcap, k_vcap_target);
        volt_ratio_ = ffd_ratio + vloop_ref_;
        break;

    case BufferCapState::kDischarging:
        // 电容电压到达危险下限 → 停止放电
        if (vcap < k_vcap_danger_low) {
            state_ = BufferCapState::kIdle;
            volt_ratio_ = ffd_ratio;
            break;
        }
        // 母线电压恢复 → 重新充电
        if (vin > k_vin_charge_threshold) {
            state_ = BufferCapState::kCharging;
            ploop_.reset();
            vloop_.reset();
            volt_ratio_ = ffd_ratio;
            break;
        }

        //TODO: 必须要求在该模式下放电，否则如果电池与负载的工作恰好使得母线电压工作在该区域（小于恢复阈值大于稳压的期望值）
        // 会导致其不断给电容充电，造成严重的过压问题
        // 故首先要保证pid的范围 > 0, 此外还需要通过电容的功率(正负)来判断电容处于充电或放电状态以决定是否要切出该模式
        // 还可以通过电容电压是否大于 k_vcap_target 来判断是否需要切出该模式

        // TODO: 放电时电容电压过低导致mos击穿问题，需要在该模式下添加保护机制

        // 母线电压环：主动放电维持 Vin=22.5V
        // error = 22.5V - Vin，Vin 低于目标时 error 为正，需要增强放电
        // 增强放电对应减小 volt_ratio（能量从电容流向母线）
        // 该模式绝对不能充电
        bus_vloop_.compute(vin, k_vin_reg_target);
        volt_ratio_ = ffd_ratio - bus_vloop_.out;
        break;
    }

    // 前馈约束，避免 vin 过小导致除零
    float ff_bound = vcap / ((vin < 15.0f) ? 15.0f : vin);
    if (volt_ratio_ > ff_bound + 0.2f) {
        volt_ratio_ = ff_bound + 0.2f;
    } else if (volt_ratio_ < ff_bound - 0.2f) {
        volt_ratio_ = ff_bound - 0.2f;
    }
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
    if (HAL_GetTick() - comm_->last_tick() > k_can_timeout_ms) {
        can_disable_cnt_++;
    } else {
        can_disable_cnt_ = 0;
    }
    can_disable_flag_ = (can_disable_cnt_ > 5);

    // TODO:仅测试
    // can_disable_flag_ = false;

    comm_->chassis2cap_msg.robot_hp = 100;

    // 缓冲电容模式保护：母线过压、母线严重欠压、电容过压、CAN断连
    bool abnormal = can_disable_flag_ ||
                    (sampler_.vin() < k_vin_low_threshold) ||
                    (sampler_.vin() > k_vin_high_threshold) ||
                    (sampler_.vcap() > k_vcap_overvolt);

    if (abnormal) {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        state_ = BufferCapState::kIdle;
        vloop_.reset();
        ploop_.reset();
        bus_vloop_.reset();
        restart_flag_ = true;
    }

    bool recovered = !can_disable_flag_ &&
                     (sampler_.vin() > k_vin_recover_low) &&
                     (sampler_.vin() < k_vin_recover_high) &&
                     (sampler_.vcap() < k_vcap_overvolt) &&
                     restart_flag_;

    if (recovered) {
        HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        restart_flag_ = false;
    }
}

void ControlTask::onTim4()
{
    can_div_cnt_++;
    if (can_div_cnt_ == 19) {
        comm_->cap2chassis_msg.cap_volt = sampler_.vcap();
        comm_->cap2chassis_msg.outpower = sampler_.pin();
        can_div_cnt_ = 0;
        comm_->send();
    }
}

void ControlTask::debugSync()
{
    // ========== AdcSampler (通过公有 getter 访问) ==========
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

    // ========== PID 控制器 (成员均为 public) ==========
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

    // ========== Buck-Boost (通过公有 getter 访问) ==========
    g_dbg_buck_duty  = converter_.buck_duty();
    g_dbg_boost_duty = converter_.boost_duty();

    // ========== CAN 通信 ==========
    g_dbg_chassis_power = comm_->chassis2cap_msg.chassis_power;
    g_dbg_cap_volt      = comm_->cap2chassis_msg.cap_volt;
    g_dbg_outpower      = comm_->cap2chassis_msg.outpower;
    g_dbg_can_last_tick = comm_->last_tick();
}

} // namespace supercap

// C链接的 HAL 定时器回调桥接
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    supercap::g_control_task.dispatch(htim);
}

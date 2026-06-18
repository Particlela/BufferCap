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

namespace supercap {

void ControlTask::init()
{
    comm_ = &g_super_cap_comm;

    // 1号板 平衡缓冲电容
    vloop_.init(0.06f, 0.00001f, 0.06f, 0.02f, -0.06f, -0.02f);
    ploop_.init(0.0f, 0.000005f, 0.06f, 0.06f, -0.06f, -0.06f);

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
    power_set_target_ = comm_->chassis2cap_msg.chassis_power;

    if (power_start_tick_ < k_soft_start_time) {
        if (power_set_ < power_set_target_) {
            power_set_ += 0.01f;
        } else {
            power_set_ -= 0.01f;
        }
        power_start_tick_++;
    } else {
        if (power_set_ < power_set_target_ - 1.50f) {
            power_set_ += 1.0f;
        } else if (power_set_ > power_set_target_ + 1.50f) {
            power_set_ -= 1.0f;
        } else {
            power_set_ = power_set_target_;
        }
    }

    if (sampler_.vcap() < 10.0f && power_set_ > 35.0f) {
        power_set_ = 35.0f;
    }

    if (power_set_ > 190.0f) power_set_ = 190.0f;
    if (power_set_ < 10.0f) power_set_ = 10.0f;
}

void ControlTask::computeControlLoop()
{
    ploop_ref_ = ploop_.compute(sampler_.pin(), power_set_);
    vloop_ref_ = vloop_.compute(sampler_.vcap(), cap_v_max_);
    cloop_ref_ = ploop_ref_;

    float vin_clamped = (sampler_.vin() < 19.0f) ? 19.0f : sampler_.vin();
    float ffd_ratio = sampler_.vcap() / vin_clamped;

    float cloop_ratio = ffd_ratio + cloop_ref_;
    float vloop_ratio = ffd_ratio + vloop_ref_;

    if (ploop_ref_ > 0.013f) {
        volt_ratio_ = (vloop_ratio <= cloop_ratio) ? vloop_ratio : cloop_ratio;
    } else {
        volt_ratio_ = cloop_ratio;

        if (sampler_.vcap() < k_uvp_trigger && !uvp_) {
            uvp_ = true;
        } else if (sampler_.vcap() > k_uvp_release && uvp_) {
            uvp_ = false;
        }

        if (uvp_) {
            volt_ratio_ = 10.5f / sampler_.vin();
        }
    }

    float ff_bound = sampler_.vcap() / sampler_.vin();
    if (volt_ratio_ > ff_bound + 0.1f) {
        volt_ratio_ = ff_bound + 0.1f;
    } else if (volt_ratio_ < ff_bound - 0.1f) {
        volt_ratio_ = ff_bound - 0.1f;
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

    comm_->chassis2cap_msg.robot_hp = 100;

    bool abnormal = (comm_->chassis2cap_msg.robot_hp == 0) ||
                    (sampler_.vin() < k_vin_low_threshold) ||
                    (sampler_.vin() > k_vin_high_threshold) ||
                    can_disable_flag_;

    if (abnormal) {
        HAL_HRTIM_WaveformOutputStop(&hhrtim1, HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 | HRTIM_OUTPUT_TE1 | HRTIM_OUTPUT_TE2);
        vloop_.reset();
        ploop_.reset();
        restart_flag_ = true;
    }

    bool recovered = (comm_->chassis2cap_msg.robot_hp != 0) &&
                     (sampler_.vin() > k_vin_recover_low) &&
                     (sampler_.vin() < k_vin_recover_high) &&
                     !can_disable_flag_ &&
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

} // namespace supercap

// C链接的 HAL 定时器回调桥接
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    supercap::g_control_task.dispatch(htim);
}

/*
 * @Author: Particlela 2480338120@qq.com
 * @Date: 2026-07-07 21:25:26
 * @LastEditors: Particlela 2480338120@qq.com
 * @LastEditTime: 2026-07-07 23:08:27
 * @FilePath: \2026BuffCapBalance\User\debug_vars.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file    debug_vars.h
 * @brief   Global debug mirror variables for Keil Watch.
 */
#ifndef DEBUG_VARS_H
#define DEBUG_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern float    g_dbg_vin;
extern float    g_dbg_cin;
extern float    g_dbg_cout;
extern float    g_dbg_vcap;
extern float    g_dbg_ccap;
extern float    g_dbg_vout;
extern float    g_dbg_vin_last;
extern float    g_dbg_vcap_last;
extern uint16_t g_dbg_raw_vin[8];
extern uint16_t g_dbg_raw_cin[8];
extern uint16_t g_dbg_raw_cout[8];
extern uint16_t g_dbg_raw_vcap[8];
extern uint16_t g_dbg_raw_ccap[8];

extern uint8_t   g_dbg_state;
extern float     g_dbg_chassis_power_target;
extern float     g_dbg_bus_power_target;
extern float     g_dbg_power_set;
extern float     g_dbg_cap_v_max;
extern float     g_dbg_volt_ratio;
extern float     g_dbg_ploop_ref;
extern float     g_dbg_vloop_ref;
extern float     g_dbg_cloop_ref;
extern float     g_dbg_bus_vloop_out;
extern bool      g_dbg_restart_flag;
extern int       g_dbg_can_disable_cnt;
extern bool      g_dbg_can_disable_flag;
extern uint16_t  g_dbg_power_start_tick;
extern uint16_t  g_dbg_can_div_cnt;
extern float     g_dbg_ffd_ratio;
extern bool      g_dbg_low_clamp_active;
extern bool      g_dbg_can_charge_cap;
extern float     g_dbg_pcap;
extern float     g_dbg_load_power_est;
extern float     g_dbg_vcap_reg_target;
extern uint32_t  g_dbg_work_tick;

extern float g_dbg_vloop_kp;
extern float g_dbg_vloop_ki;
extern float g_dbg_vloop_pout;
extern float g_dbg_vloop_iout;
extern float g_dbg_vloop_out;
extern float g_dbg_vloop_err[2];

extern float g_dbg_high_vloop_kp;
extern float g_dbg_high_vloop_ki;
extern float g_dbg_high_vloop_pout;
extern float g_dbg_high_vloop_iout;
extern float g_dbg_high_vloop_out;
extern float g_dbg_high_vloop_err[2];

extern float g_dbg_low_vloop_kp;
extern float g_dbg_low_vloop_ki;
extern float g_dbg_low_vloop_pout;
extern float g_dbg_low_vloop_iout;
extern float g_dbg_low_vloop_out;
extern float g_dbg_low_vloop_err[2];

extern float g_dbg_ploop_kp;
extern float g_dbg_ploop_ki;
extern float g_dbg_ploop_pout;
extern float g_dbg_ploop_iout;
extern float g_dbg_ploop_out;
extern float g_dbg_ploop_err[2];

extern float g_dbg_busvloop_kp;
extern float g_dbg_busvloop_ki;
extern float g_dbg_busvloop_pout;
extern float g_dbg_busvloop_iout;
extern float g_dbg_busvloop_out;
extern float g_dbg_busvloop_err[2];

extern float g_dbg_buck_duty;
extern float g_dbg_boost_duty;

extern float    g_dbg_battery_power;
extern uint32_t g_dbg_can_last_tick;

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_VARS_H */

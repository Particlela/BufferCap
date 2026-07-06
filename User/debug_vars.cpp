/*
 * @Author: 世界最低峰. 17091062+GrennBee@user.noreply.gitee.com
 * @Date: 2026-06-18 21:48:07
 * @LastEditors: Particlela 2480338120@qq.com
 * @LastEditTime: 2026-07-06 23:14:13
 * @FilePath: \2026BuffCapBalance\User\debug_vars.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 * @file    debug_vars.cpp
 * @brief   供 Keil 调试器观测的全局镜像变量定义
 */
#include "debug_vars.h"

/* ==================================================================
 * AdcSampler
 * ================================================================*/
float    g_dbg_vin          = 0.0f;
float    g_dbg_cin          = 0.0f;
float    g_dbg_cout         = 0.0f;
float    g_dbg_vcap         = 0.0f;
float    g_dbg_ccap         = 0.0f;
float    g_dbg_vout         = 0.0f;
float    g_dbg_vin_last     = 0.0f;
float    g_dbg_vcap_last    = 0.0f;
uint16_t g_dbg_raw_vin[8]   = {0};
uint16_t g_dbg_raw_cin[8]   = {0};
uint16_t g_dbg_raw_cout[8]  = {0};
uint16_t g_dbg_raw_vcap[8]  = {0};
uint16_t g_dbg_raw_ccap[8]  = {0};

/* ==================================================================
 * ControlTask
 * ================================================================*/
uint8_t  g_dbg_state              = 0;
float    g_dbg_bus_power_target   = 0.0f;
float    g_dbg_power_set          = 0.0f;
float    g_dbg_cap_v_max          = 0.0f;
float    g_dbg_volt_ratio         = 0.0f;
float    g_dbg_ploop_ref          = 0.0f;
float    g_dbg_vloop_ref          = 0.0f;
float    g_dbg_cloop_ref          = 0.0f;
float    g_dbg_bus_vloop_out      = 0.0f;
bool     g_dbg_restart_flag       = false;
int      g_dbg_can_disable_cnt    = 0;
bool     g_dbg_can_disable_flag   = false;
uint16_t g_dbg_power_start_tick   = 0;
uint16_t g_dbg_can_div_cnt        = 0;
float    g_dbg_ffd_ratio          = 0.0f;
bool     g_dbg_low_clamp_active   = false;
bool     g_dbg_can_charge_cap     = false;
float    g_dbg_pcap               = 0.0f;
float    g_dbg_load_power_est     = 0.0f;
float    g_dbg_vcap_reg_target    = 0.0f;
uint32_t g_dbg_work_tick         = 0;

/* ==================================================================
 * PID 控制器
 * ================================================================*/
/* --- 电压环 --- */
float g_dbg_vloop_kp       = 0.0f;
float g_dbg_vloop_ki       = 0.0f;
float g_dbg_vloop_pout     = 0.0f;
float g_dbg_vloop_iout     = 0.0f;
float g_dbg_vloop_out      = 0.0f;
float g_dbg_vloop_err[2]   = {0.0f, 0.0f};

/* --- 楂樺帇鐢靛帇鐜?--- */
float g_dbg_high_vloop_kp       = 0.0f;
float g_dbg_high_vloop_ki       = 0.0f;
float g_dbg_high_vloop_pout     = 0.0f;
float g_dbg_high_vloop_iout     = 0.0f;
float g_dbg_high_vloop_out      = 0.0f;
float g_dbg_high_vloop_err[2]   = {0.0f, 0.0f};

/* --- 浣庡帇鐢靛帇鐜?--- */
float g_dbg_low_vloop_kp       = 0.0f;
float g_dbg_low_vloop_ki       = 0.0f;
float g_dbg_low_vloop_pout     = 0.0f;
float g_dbg_low_vloop_iout     = 0.0f;
float g_dbg_low_vloop_out      = 0.0f;
float g_dbg_low_vloop_err[2]   = {0.0f, 0.0f};

/* --- 功率环 --- */
float g_dbg_ploop_kp       = 0.0f;
float g_dbg_ploop_ki       = 0.0f;
float g_dbg_ploop_pout     = 0.0f;
float g_dbg_ploop_iout     = 0.0f;
float g_dbg_ploop_out      = 0.0f;
float g_dbg_ploop_err[2]   = {0.0f, 0.0f};

/* --- 母线电压环 --- */
float g_dbg_busvloop_kp       = 0.0f;
float g_dbg_busvloop_ki       = 0.0f;
float g_dbg_busvloop_pout     = 0.0f;
float g_dbg_busvloop_iout     = 0.0f;
float g_dbg_busvloop_out      = 0.0f;
float g_dbg_busvloop_err[2]   = {0.0f, 0.0f};

/* ==================================================================
 * Buck-Boost
 * ================================================================*/
float g_dbg_buck_duty   = 0.0f;
float g_dbg_boost_duty  = 0.0f;

/* ==================================================================
 * CAN 通信
 * ================================================================*/
float    g_dbg_battery_power = 0.0f;
uint32_t g_dbg_can_last_tick = 0;

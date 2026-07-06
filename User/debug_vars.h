/**
 * @file    debug_vars.h
 * @brief   供 Keil 调试器观测的全局镜像变量
 *
 * 所有变量均在 debug_vars_sync() 中从 supercap::g_control_task 同步，
 * 在 TIM2 中断中自动更新，可在 Keil Watch 窗口直接观察。
 */
#ifndef DEBUG_VARS_H
#define DEBUG_VARS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================================================================
 * AdcSampler 镜像变量
 * ================================================================*/
extern float    g_dbg_vin;          ///< 母线电压 (V)
extern float    g_dbg_cin;          ///< 输入电流 (A)
extern float    g_dbg_cout;         ///< 输出电流 (A)
extern float    g_dbg_vcap;         ///< 电容电压 (V)
extern float    g_dbg_ccap;         ///< 电容电流 (A)
extern float    g_dbg_vout;         ///< 变换器输出电压 (V)
extern float    g_dbg_vin_last;     ///< 上一拍母线电压 (滤波用)
extern float    g_dbg_vcap_last;    ///< 上一拍电容电压 (滤波用)

extern uint16_t g_dbg_raw_vin[8];   ///< ADC 原始采样 – 输入电压
extern uint16_t g_dbg_raw_cin[8];   ///< ADC 原始采样 – 输入电流
extern uint16_t g_dbg_raw_cout[8];  ///< ADC 原始采样 – 输出电流
extern uint16_t g_dbg_raw_vcap[8];  ///< ADC 原始采样 – 电容电压
extern uint16_t g_dbg_raw_ccap[8];  ///< ADC 原始采样 – 电容电流

/* ==================================================================
 * ControlTask 镜像变量
 * ================================================================*/
extern uint8_t   g_dbg_state;               ///< BufferCapState 状态
extern float     g_dbg_bus_power_target;    ///< 母线功率目标
extern float     g_dbg_power_set;           ///< 软启动功率设定
extern float     g_dbg_cap_v_max;           ///< 电容最大电压限制
extern float     g_dbg_volt_ratio;          ///< 当前占空比系数
extern float     g_dbg_ploop_ref;           ///< 功率环输出
extern float     g_dbg_vloop_ref;           ///< 电压环输出
extern float     g_dbg_cloop_ref;           ///< 电流环 (取小) 输出
extern bool      g_dbg_restart_flag;        ///< 保护重启标志
extern int       g_dbg_can_disable_cnt;     ///< CAN 断连计数器
extern bool      g_dbg_can_disable_flag;    ///< CAN 断连标志
extern uint16_t  g_dbg_power_start_tick;    ///< 软启动计时
extern uint16_t  g_dbg_can_div_cnt;         ///< CAN 发送分频
extern float     g_dbg_ffd_ratio;           ///< 当前 FFD 占空比系数
extern bool      g_dbg_startup_charging;    ///< 启动充电模式 (CAN 刚连上且 vcap < k_vcap_min)
extern float     g_dbg_vcap_reg_target;     ///< 稳压目标值 (k_vcap_high 或 k_vcap_low)
extern uint32_t  g_dbg_work_tick;           ///< CAN 正常工作时间 (ms)，断连清零

/* ==================================================================
 * PID 控制器镜像变量
 * ================================================================*/
/* --- 电压环 (vloop_) --- */
extern float g_dbg_vloop_kp;
extern float g_dbg_vloop_ki;
extern float g_dbg_vloop_pout;
extern float g_dbg_vloop_iout;
extern float g_dbg_vloop_out;
extern float g_dbg_vloop_err[2];

/* --- 功率环 (ploop_) --- */
extern float g_dbg_ploop_kp;
extern float g_dbg_ploop_ki;
extern float g_dbg_ploop_pout;
extern float g_dbg_ploop_iout;
extern float g_dbg_ploop_out;
extern float g_dbg_ploop_err[2];

/* --- 母线电压环 (bus_vloop_) --- */
extern float g_dbg_busvloop_kp;
extern float g_dbg_busvloop_ki;
extern float g_dbg_busvloop_pout;
extern float g_dbg_busvloop_iout;
extern float g_dbg_busvloop_out;
extern float g_dbg_busvloop_err[2];

/* ==================================================================
 * Buck-Boost 变换器镜像变量
 * ================================================================*/
extern float g_dbg_buck_duty;       ///< Buck 占空比
extern float g_dbg_boost_duty;     ///< Boost 占空比

/* ==================================================================
 * CAN 通信镜像变量
 * ================================================================*/
extern float    g_dbg_battery_power;   ///< 从电池吸收的功率指令 (W)，来自底盘
extern uint32_t g_dbg_can_last_tick;   ///< 最近 CAN 接收时间戳

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_VARS_H */

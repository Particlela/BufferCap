/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2026-01-18 16:23:59
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-01-22 22:51:55
 * @FilePath: \MDK-ARMc:\Users\pc\Desktop\26HW-SuperCap\SuperCap\User\types.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __TYPES_H__
#define __TYPES_H__

#include "stdint.h"
#include "stdbool.h"

#define SOFT_START_TIME 5000                                       // 软启动开关 5000 ms
#define shift_duty 0.96f                                           // 最大占空比
#define shift_duty_2 2 * shift_duty                                // 最大占空比 * 2
#define DP_PWM_PER 27200                                           // 定时器HRTIM周期
#define MAX_PWM_CMP (uint32_t)(shift_duty * DP_PWM_PER)            // PWM最大比较值
#define MIN_PWM_CMP (uint32_t)((1 - shift_duty) * DP_PWM_PER)      // PWM最小比较值
#define max_volt_ratio 1.32f                                       // 最大电压比值
#define min_volt_ratio 0.05f                                       // 最小电压比值 
#define buf_len 8                                                  // 缓冲区长度
#define I_BIAS 2047                                                // 电流偏移量
#define adc_maxvalu 4095                                           // adc最大值
#define adc_vref 3.0f                                              // adc参考电压
#define adc_conv_fact adc_vref / adc_maxvalu                       // adc转换系数
#define I_GAIN 0.05f                                               // 电流增益
#define V_GAIN 20.0f                                               // 电压增益
#define R_SAMP_2mR 500.0f                                          // 2mR采样电阻倒数

typedef struct
{
uint16_t vin[buf_len];                                             // adc采样原始输入电压数据
uint16_t cin[buf_len];                                             // adc采样原始输入电流数据
uint16_t cout[buf_len];                                            // adc采样原始输出电流数据
uint16_t vcap[buf_len];                                            // adc采样原始电容电压数据
uint16_t ccap[buf_len];                                            // adc采样原始电容电流数据
uint16_t cnt;				  					                   								 // 分频计数器
} Sample_struct_t;                                                 // adc采样数据结构体

typedef struct
{
float vin;                                                         // 输入电压
float vin_last;                                                    // 上次输入电压
float cin;                                                         // 输入电流
float pin;                                                         // 输入功率
float vout;                                                        // 输出电压
float cout;                                                        // 输出电流
float pout;                                                        // 输出功率
float vcap;                                                        // 电容电压
float vcap_last;                                                   // 上次电容电压  
float ccap;                                                        // 电容电流
uint16_t n;                                                        // 板子编号
} Value_struct_t;                                                  // 环路计算数据结构体

typedef struct
{
float buck_duty;                                                   // buck半桥占空比
float boost_duty;                                                  // boost半桥占空比
float power_set;                                                   // 设定输入功率
float power_set_target;                                            // 设定输入功率目标值
float cmax;                            					                   // 最大电流
float cap_v_max;                                                   // 电容最大电压
float ploop_ref;                                                   // 功率环输出参考值
float vloop_ref;                                                   // 电压环输出参考值
float cloop_ref;                                                   // 电流环输出参考值
float vloop_ratio;                                                 // 电压环输出比值
float cloop_ratio;                                                 // 电流环输出比值
float volt_ratio;                                                  // 输出电压与输入电压比值
bool uvp;														   														 // 欠压保护
float buff_power;                                                  // 缓冲能量 
} Control_struct_t;                                                // 控制结构体


typedef struct
{
float kp;
float ki;
float pout;
float iout;
float out;
float out_max;
float out_min;
float iout_max;
float iout_min;
float error[2];
} PID_struct_t;                                                    // PID结构体

extern Sample_struct_t Sample;                                     // adc采样数据结构体实例
extern Value_struct_t Value;                                       // 环路计算数据结构体实例
extern Control_struct_t Control;                                   // 控制结构体实例
extern PID_struct_t vloop;                                         // 电压环结构体实例
extern PID_struct_t ploop;                                         // 功率环结构体实例

extern void PI_init(PID_struct_t *pid, float kp, float ki, float out_max, float iout_max, float out_min, float iout_min); 
#endif
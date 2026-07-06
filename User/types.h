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

#ifdef __cplusplus
extern "C" {
#endif

#define SOFT_START_TIME 5000                                       // 软启动开关 5000 ms
#define buf_len 8                                                  // 缓冲区长度
#define I_BIAS 2047                                                // 电流偏移量
#define adc_maxvalu 4095                                           // adc最大值
#define adc_vref 3.0f                                              // adc参考电压
#define adc_conv_fact adc_vref / adc_maxvalu                       // adc转换系数
#define I_GAIN 0.05f                                               // 电流增益
#define V_GAIN 20.0f                                               // 电压增益
#define R_SAMP_2mR 500.0f                                          // 2mR采样电阻倒数
#define R_SAMP_1mR 1000.0f                                         // 1mR采样电阻倒数

typedef struct
{
    uint16_t vin[buf_len];                                         // adc采样原始输入电压数据
    uint16_t cin[buf_len];                                         // adc采样原始输入电流数据
    uint16_t cout[buf_len];                                        // adc采样原始输出电流数据
    uint16_t vcap[buf_len];                                        // adc采样原始电容电压数据
    uint16_t ccap[buf_len];                                        // adc采样原始电容电流数据
    uint16_t cnt;                                                  // 分频计数器
} Sample_struct_t;                                                 // adc采样数据结构体

#ifdef __cplusplus
}
#endif

#endif
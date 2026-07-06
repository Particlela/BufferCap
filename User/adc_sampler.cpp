/**
 * @file    adc_sampler.cpp
 * @brief   ADC 采样、滤波与校准类实现
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"
#include "dma.h"

#ifdef __cplusplus
}
#endif

#include "adc_sampler.hpp"

namespace supercap {

void AdcSampler::startDMA()
{
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED);

    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(sample_.vin), k_buf_len);
    HAL_ADC_Start_DMA(&hadc2, reinterpret_cast<uint32_t*>(sample_.cout), k_buf_len);
    HAL_ADC_Start_DMA(&hadc3, reinterpret_cast<uint32_t*>(sample_.vcap), k_buf_len);
    HAL_ADC_Start_DMA(&hadc4, reinterpret_cast<uint32_t*>(sample_.ccap), k_buf_len);
    HAL_ADC_Start_DMA(&hadc5, reinterpret_cast<uint32_t*>(sample_.cin), k_buf_len);
}

void AdcSampler::update()
{
    vin_  = fitter(sample_.vin)  * k_adc_conv_fact * k_v_gain;
    cin_  = (fitter(sample_.cin)  - k_i_bias) * k_adc_conv_fact * k_i_gain * k_r_samp_1mr;
    cout_ = (fitter(sample_.cout) - k_i_bias) * k_adc_conv_fact * k_i_gain * k_r_samp_1mr;
    vcap_ =  fitter(sample_.vcap) * k_adc_conv_fact * k_v_gain;
    ccap_ = (fitter(sample_.ccap) - k_i_bias) * k_adc_conv_fact * k_i_gain * k_r_samp_1mr;
}

void AdcSampler::applyComplementaryFilter(bool soft_starting)
{
    if (vin_ < 0.1f) vin_ = 0.1f;
    if (vcap_ < 0.1f) vcap_ = 0.1f;

    if (soft_starting) {
        vin_last_ = vin_;
        vcap_last_ = vcap_;
    } else {
        vin_ = 0.8f * vin_ + 0.2f * vin_last_;
        vin_last_ = vin_;
        vcap_ = 0.8f * vcap_ + 0.2f * vcap_last_;
        vcap_last_ = vcap_;
    }
}

void AdcSampler::calibrate(uint16_t board_id)
{
    switch (board_id) {
        case 0:
            // 默认不校准
            break;
        case 1:
            vin_  = calibrate_value(vin_,  25.010, 24.759, 21.010, 20.783);
            vcap_ = calibrate_value(vcap_, 14.588, 14.298, 11.926, 11.833);
            ccap_ = calibrate_value(ccap_, 1.531,  1.453,  4.966,  4.983);
            cin_  = calibrate_value(cin_,  1.000,  0.922,  3.000,  2.943);
            cout_ = calibrate_value(cout_, 0.904,  0.940,  3.904,  4.025);
            break;
        case 2:
            vin_  = calibrate_value(vin_,  25.010, 24.833, 20.010, 19.848);
            vcap_ = calibrate_value(vcap_, 13.974, 13.950, 10.984, 11.043);
            ccap_ = calibrate_value(ccap_, 5.004,  5.900,  6.370,  6.826);
            cin_  = calibrate_value(cin_,  3.000,  3.058,  3.780,  3.869);
            cout_ = calibrate_value(cout_, 2.820,  2.841,  3.544,  3.553);
            break;
        default:
            break;
    }
}

float AdcSampler::fitter(const uint16_t *data)
{
    uint16_t sum = 0;
    for (int i = 0; i < k_buf_len; i++) {
        sum += data[i];
    }
    return sum * (1.0f / static_cast<float>(k_buf_len));
}

float AdcSampler::calibrate_value(float p, double real1, double get1, double real2, double get2)
{
    double a = (real2 - real1) / (get2 - get1);
    double b = real1 - get1 * a;
    return static_cast<float>(p * a + b);
}

} // namespace supercap

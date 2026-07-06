/**
 * @file    adc_sampler.hpp
 * @brief   ADC 采样、滤波与校准类
 */
#ifndef ADC_SAMPLER_HPP
#define ADC_SAMPLER_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "adc.h"
#include "dma.h"
#include "types.h"

#ifdef __cplusplus
}
#endif

namespace supercap {

class AdcSampler {
public:
    static constexpr uint16_t k_buf_len = buf_len;
    static constexpr float k_adc_conv_fact = adc_conv_fact;
    static constexpr float k_i_gain = I_GAIN;
    static constexpr float k_v_gain = V_GAIN;
    static constexpr float k_r_samp_2mr = R_SAMP_2mR;
    static constexpr float k_r_samp_1mr = R_SAMP_1mR;
    static constexpr uint16_t k_i_bias = I_BIAS;

    void startDMA();
    void update();
    void calibrate(uint16_t board_id);
    void applyComplementaryFilter(bool soft_starting);

    float vin() const { return vin_; }
    float cin() const { return cin_; }
    float cout() const { return cout_; }
    float vcap() const { return vcap_; }
    float ccap() const { return ccap_; }
    float vin_last() const { return vin_last_; }
    float vcap_last() const { return vcap_last_; }
    float pin() const { return vin_ * cin_; }
    float pout() const { return vout_ * cout_; }

    void set_vout(float vout) { vout_ = vout; }
    float vout() const { return vout_; }

    // 保留原始DMA缓冲区访问能力
    Sample_struct_t* sample_buf() { return &sample_; }
    const Sample_struct_t* sample_buf() const { return &sample_; }

private:
    static float fitter(const uint16_t *data);
    static float calibrate_value(float p, double real1, double get1, double real2, double get2);

    Sample_struct_t sample_ = {0};
    float vin_ = 0.0f;
    float cin_ = 0.0f;
    float cout_ = 0.0f;
    float vcap_ = 0.0f;
    float ccap_ = 0.0f;
    float vout_ = 0.0f;
    float vin_last_ = 0.0f;
    float vcap_last_ = 0.0f;
};

} // namespace supercap

#endif // ADC_SAMPLER_HPP

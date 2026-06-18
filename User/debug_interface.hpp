/**
 * @file    debug_interface.hpp
 * @brief   Vofa 调试接口类
 */
#ifndef DEBUG_INTERFACE_HPP
#define DEBUG_INTERFACE_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

#ifdef __cplusplus
}
#endif

#include "adc_sampler.hpp"
#include "super_cap_comm.hpp"

namespace supercap {

class ControlTask; // 前向声明

class DebugInterface {
public:
    void update(const AdcSampler& sampler, const ControlTask& ctrl, const SuperCapComm& comm);
    void send();

private:
    struct {
        float data[13];
        char tail[4];
    } buf_ = {.tail = {0x00, 0x00, 0x80, 0x7F}};
};

} // namespace supercap

#endif // DEBUG_INTERFACE_HPP

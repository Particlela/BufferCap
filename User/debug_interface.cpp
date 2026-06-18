/**
 * @file    debug_interface.cpp
 * @brief   Vofa 调试接口类实现
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"

#ifdef __cplusplus
}
#endif

#include "debug_interface.hpp"
#include "control_task.hpp"

namespace supercap {

void DebugInterface::update(const AdcSampler& sampler, const ControlTask& ctrl, const SuperCapComm& comm)
{
    buf_.data[0] = sampler.vin();
    buf_.data[1] = sampler.cin();
    buf_.data[2] = ctrl.ploop_ref();
    buf_.data[3] = sampler.vcap();
    buf_.data[4] = sampler.pin();
    buf_.data[5] = ctrl.power_set();
    buf_.data[6] = ctrl.power_set_target();
    buf_.data[7] = ctrl.cloop_ref();
    buf_.data[8] = comm.chassis2cap_msg.chassis_power;
    buf_.data[9] = comm.chassis2cap_msg.pwr_buf;
    buf_.data[10] = static_cast<float>(comm.chassis2cap_msg.robot_hp);
    buf_.data[11] = ctrl.can_disable_flag() ? 1.0f : 0.0f;
    buf_.data[12] = static_cast<float>(ctrl.can_disable_cnt());
}

void DebugInterface::send()
{
    HAL_UART_Transmit_DMA(&huart1, reinterpret_cast<uint8_t*>(&buf_), sizeof(buf_));
}

} // namespace supercap

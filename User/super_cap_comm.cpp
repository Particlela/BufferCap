/**
 * @file    super_cap_comm.cpp
 * @brief   超级电容与底盘 CAN 通信类实现
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
}
#endif

#include "super_cap_comm.hpp"
#include "global_instances.hpp"

namespace supercap {

static uint8_t tx_data[8] = {0};
static FDCAN_RxHeaderTypeDef rx_header = {0};

void SuperCapComm::init()
{
    filterInit();
    cap2chassis_msg = SuperCap2ChassisMsg{};
    chassis2cap_msg = Chassis2SuperCapMsg{};
}

void SuperCapComm::filterInit()
{
    FDCAN_FilterTypeDef sFilterConfig = {0};

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = kRxId;
    sFilterConfig.FilterID2 = kRxId;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, FDCAN_INTERRUPT_LINE0);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);
    HAL_FDCAN_Start(&hfdcan2);
}

bool SuperCapComm::decode(const uint8_t rx_data[8], uint32_t rx_msg_std_id)
{
    if (rx_msg_std_id != kRxId) {
        return false;
    }
    chassis2cap_msg.pwr_buf = static_cast<uint16_t>((rx_data[0] << 8) | rx_data[1]);
    chassis2cap_msg.pwr_limit = static_cast<uint16_t>((rx_data[2] << 8) | rx_data[3]);
    chassis2cap_msg.robot_hp = static_cast<uint16_t>((rx_data[4] << 8) | rx_data[5]);
    chassis2cap_msg.chassis_power = static_cast<float>((static_cast<uint16_t>((rx_data[6] << 8) | rx_data[7])) / 100.0f);
    return true;
}

void SuperCapComm::encode(uint8_t tx_data[8]) const
{
    uint16_t remain_pct_tx = static_cast<uint16_t>(cap2chassis_msg.remain_pct * 100);
    uint16_t outpower_tx = static_cast<uint16_t>(cap2chassis_msg.outpower * 100);
    uint16_t cap_volt_tx = static_cast<uint16_t>(cap2chassis_msg.cap_volt * 100);

    tx_data[0] = static_cast<uint8_t>(remain_pct_tx >> 8);
    tx_data[1] = static_cast<uint8_t>(remain_pct_tx);
    tx_data[2] = static_cast<uint8_t>(cap2chassis_msg.volt_src);
    tx_data[3] = static_cast<uint8_t>(outpower_tx >> 8);
    tx_data[4] = static_cast<uint8_t>(outpower_tx);
    tx_data[5] = static_cast<uint8_t>(cap_volt_tx >> 8);
    tx_data[6] = static_cast<uint8_t>(cap_volt_tx);
    tx_data[7] = 0;
}

void SuperCapComm::updateTxMsg(const SuperCap2ChassisMsg& msg)
{
    cap2chassis_msg = msg;
}

Chassis2SuperCapMsg SuperCapComm::getRxMsg() const
{
    return chassis2cap_msg;
}

void SuperCapComm::send()
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    tx_header.Identifier = kTxId;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    encode(tx_data);
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data);
}

void SuperCapComm::rxFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    uint8_t rx_data[8] = {0};

    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

    if (g_super_cap_comm.decode(rx_data, rx_header.Identifier)) {
        if (g_super_cap_comm.chassis2cap_msg.pwr_limit > 130) {
            g_super_cap_comm.chassis2cap_msg.pwr_limit = 130;
        }
        if (g_super_cap_comm.chassis2cap_msg.pwr_limit < 10) {
            g_super_cap_comm.chassis2cap_msg.pwr_limit = 10;
        }
        g_super_cap_comm.last_tick_ = HAL_GetTick();
    }
}

void SuperCapComm::errorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
    if (hfdcan == &hfdcan2 && (ErrorStatusITs & FDCAN_IT_BUS_OFF)) {
        FDCAN_ProtocolStatusTypeDef protocol_status = {0};
        HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status);
        if (protocol_status.BusOff) {
            HAL_FDCAN_Init(hfdcan);
            g_super_cap_comm.filterInit();
        }
    }
}

} // namespace supercap

// C链接的 HAL 回调桥接
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    supercap::SuperCapComm::rxFifoCallback(hfdcan, RxFifo0ITs);
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
    supercap::SuperCapComm::errorStatusCallback(hfdcan, ErrorStatusITs);
}

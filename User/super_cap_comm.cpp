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

/// @brief 解码底盘 → 超级电容消息
/// @details battery_power: 大端 uint16, 缩放 ×10, 范围 0 ~ 6553.5 W
bool SuperCapComm::decode(const uint8_t rx_data[8], uint32_t rx_msg_std_id)
{
    if (rx_msg_std_id != kRxId) {
        return false;
    }
    uint16_t power_raw = static_cast<uint16_t>((rx_data[0] << 8) | rx_data[1]);
    chassis2cap_msg.battery_power = static_cast<float>(power_raw) / 10.0f;
    return true;
}

/// @brief 编码超级电容 → 底盘消息
/// @details 5 个 12-bit 信号 + 4-bit 保留位，共 64 bit (8 字节)
void SuperCapComm::encode(uint8_t tx_data[8]) const
{
    // ---------- 原始整数值计算 ----------
    // remain_pct: 0~100% → ×10 → 0~1000 (12 bit)
    uint16_t remain_raw = static_cast<uint16_t>(cap2chassis_msg.remain_pct * 10.0f);
    // vin: 0~30V → ×100 → 0~3000 (12 bit)
    uint16_t vin_raw = static_cast<uint16_t>(cap2chassis_msg.vin * 100.0f);
    // vcap: 0~30V → ×100 → 0~3000 (12 bit)
    uint16_t vcap_raw = static_cast<uint16_t>(cap2chassis_msg.vcap * 100.0f);
    // ccap: -80~80A → ×20 → -1600~1600 → +80 偏移 → 0~3200 (12 bit)
    uint16_t ccap_raw = static_cast<uint16_t>((cap2chassis_msg.ccap + 80.0f) * 20.0f);
    // cin: -30~80A → ×20 → -600~1600 → +30 偏移 → 0~2200 (12 bit)
    uint16_t cin_raw = static_cast<uint16_t>((cap2chassis_msg.cin + 30.0f) * 20.0f);

    // ---------- 限幅 ----------
    if (remain_raw > 1000) remain_raw = 1000;
    if (vin_raw > 3000) vin_raw = 3000;
    if (vcap_raw > 3000) vcap_raw = 3000;
    if (ccap_raw > 3200) ccap_raw = 3200;
    if (cin_raw > 2200) cin_raw = 2200;

    // ---------- 按位打包 (大端序, MSB first) ----------
    // bit  0~11: remain_pct
    tx_data[0] = static_cast<uint8_t>(remain_raw >> 4);                                    // remain[11:4]
    tx_data[1] = static_cast<uint8_t>(((remain_raw & 0x0F) << 4) | (vin_raw >> 8));        // remain[3:0] | vin[11:8]
    // bit 12~23: vin
    tx_data[2] = static_cast<uint8_t>(vin_raw & 0xFF);                                     // vin[7:0]
    // bit 24~35: vcap
    tx_data[3] = static_cast<uint8_t>(vcap_raw >> 4);                                      // vcap[11:4]
    tx_data[4] = static_cast<uint8_t>(((vcap_raw & 0x0F) << 4) | (ccap_raw >> 8));         // vcap[3:0] | ccap[11:8]
    // bit 36~47: ccap
    tx_data[5] = static_cast<uint8_t>(ccap_raw & 0xFF);                                    // ccap[7:0]
    // bit 48~59: cin
    tx_data[6] = static_cast<uint8_t>(cin_raw >> 4);                                       // cin[11:4]
    tx_data[7] = static_cast<uint8_t>((cin_raw & 0x0F) << 4);                              // cin[3:0] | reserved[3:0]=0
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

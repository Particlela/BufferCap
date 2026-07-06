/**
 * @file    super_cap_comm.hpp
 * @brief   超级电容与底盘 CAN 通信类
 *
 * 通信协议说明：
 *
 * [RX] Chassis → SuperCap (kRxId, 8 字节)
 *   字节 0-1: battery_power × 10 (大端 uint16), 范围 0 ~ 6553.5 W
 *   字节 2-7: 保留 (填 0)
 *
 * [TX] SuperCap → Chassis (kTxId, 8 字节, 64 bit)
 *   ┌─────────────┬──────────┬─────────┬────────┬───────────────┬──────────┐
 *   │ 信号        │ 物理范围 │ 缩放系数 │ 偏移量 │ 原始整数范围  │ 位偏移   │
 *   ├─────────────┼──────────┼─────────┼────────┼───────────────┼──────────┤
 *   │ remain_pct  │ 0~100 %  │ ×10     │ 无     │ 0 ~ 1000      │ bit 0~11 │
 *   │ vin         │ 0~30 V   │ ×100    │ 无     │ 0 ~ 3000      │ bit 12~23│
 *   │ vcap        │ 0~30 V   │ ×100    │ 无     │ 0 ~ 3000      │ bit 24~35│
 *   │ ccap        │ -80~80 A │ ×20     │ +80    │ 0 ~ 3200      │ bit 36~47│
 *   │ cin         │ -30~80 A │ ×20     │ +30    │ 0 ~ 2200      │ bit 48~59│
 *   │ 保留位      │ —        │ —       │ —      | 填 0          │ bit 60~63│
 *   └─────────────┴──────────┴─────────┴────────┴───────────────┴──────────┘
 */
#ifndef SUPER_CAP_COMM_HPP
#define SUPER_CAP_COMM_HPP

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"

#ifdef __cplusplus
}
#endif

namespace supercap {

/// @brief 底盘 → 超级电容 接收消息
struct Chassis2SuperCapMsg {
    float battery_power = 0.0f;  ///< 从电池吸收的功率 (W)，本质与底盘所需功率一致
};

/// @brief 超级电容 → 底盘 发送消息
struct SuperCap2ChassisMsg {
    float remain_pct = 0.0f;  ///< 剩余电量百分比 (0~100 %)
    float vin        = 0.0f;  ///< 输入电压 / 母线电压 (V)
    float vcap       = 0.0f;  ///< 电容电压 (V)
    float ccap       = 0.0f;  ///< 电容电流 (A)，充电为正、放电为负
    float cin        = 0.0f;  ///< 电源输入电流 (A)，从电池取电为正
};

class SuperCapComm {
public:
    static constexpr uint32_t kTxId = 0x112;  ///< TX CAN ID (SuperCap → Chassis)
    static constexpr uint32_t kRxId = 0x111;  ///< RX CAN ID (Chassis → SuperCap)

    void init();

    bool decode(const uint8_t rx_data[8], uint32_t rx_msg_std_id);
    void encode(uint8_t tx_data[8]) const;
    void updateTxMsg(const SuperCap2ChassisMsg& msg);
    Chassis2SuperCapMsg getRxMsg() const;

    void send();

    /// @brief HAL RX FIFO0 回调（C 链接桥接）
    static void rxFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
    /// @brief HAL 错误状态回调（C 链接桥接）
    static void errorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs);

    /// @brief 允许外部访问最新数据
    Chassis2SuperCapMsg chassis2cap_msg;
    SuperCap2ChassisMsg cap2chassis_msg;

    uint32_t last_tick() const { return last_tick_; }

private:
    void filterInit();

    uint32_t last_tick_ = 0;
};

} // namespace supercap

#endif // SUPER_CAP_COMM_HPP

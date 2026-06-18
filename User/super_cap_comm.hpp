/**
 * @file    super_cap_comm.hpp
 * @brief   超级电容与底盘 CAN 通信类
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

enum class PwrSrc : uint8_t {
    kSupCap = 0,
    kJudge = 1,
};

struct Chassis2SuperCapMsg {
    uint16_t pwr_buf = 0;      ///< 剩余缓冲能量
    uint16_t pwr_limit = 0;    ///< 当前功率上限
    uint16_t robot_hp = 0;     ///< 机器人剩余血量
    float chassis_power = 0.0f;///< 底盘所需功率
};

struct SuperCap2ChassisMsg {
    float remain_pct = 0.0f;   ///< 剩余电量百分比
    float outpower = 0.0f;     ///< 输入电压
    PwrSrc volt_src = PwrSrc::kJudge; ///< 电压来源
    float cap_volt = 0.0f;     ///< 电容电压
};

class SuperCapComm {
public:
    static constexpr uint32_t kTxId = 0x114;
    static constexpr uint32_t kRxId = 0x113;

    void init();

    bool decode(const uint8_t rx_data[8], uint32_t rx_msg_std_id);
    void encode(uint8_t tx_data[8]) const;
    void updateTxMsg(const SuperCap2ChassisMsg& msg);
    Chassis2SuperCapMsg getRxMsg() const;

    void send();

    // HAL 回调（C链接桥接）
    static void rxFifoCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
    static void errorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs);

    // 允许外部访问最新数据
    Chassis2SuperCapMsg chassis2cap_msg;
    SuperCap2ChassisMsg cap2chassis_msg;

    uint32_t last_tick() const { return last_tick_; }

private:
    void filterInit();

    uint32_t last_tick_ = 0;
};

} // namespace supercap

#endif // SUPER_CAP_COMM_HPP

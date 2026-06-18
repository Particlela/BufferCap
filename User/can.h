/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-05-07 16:51:44
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2026-01-26 17:03:29
 * @FilePath: \code -f334改\USER\cap_com.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef CAP_COM_
#define CAP_COM_
#include <stdbool.h>

#include "main.h"
#ifdef __cplusplus
extern "C"
{
#endif
  typedef enum
  {
    kSupCap,
    kJudge,
  } PweSrc_e;

  typedef struct
  {
    uint16_t pwr_buf;    ///< 剩余缓冲能量
    uint16_t pwr_limit;  ///< 当前功率上限
    uint16_t robot_hp;   ///< 机器人剩余血量
    float chassis_power; ///< 底盘所需功率
  } Chassis2SuperCapMsg_t;

  typedef struct
  {
    float remain_pct  ;            ///< 剩余电量百分比
    float outpower;                   ///< 输入电压
    PweSrc_e volt_src;           ///< 电压来源
    float cap_volt;
  } SuperCap2ChassisMsg_t;

  typedef struct SuperCapComm SuperCapComm_t;
  struct SuperCapComm
  {
    void (*encode)(SuperCapComm_t *this, uint8_t tx_data[8]);

    bool (*decode)(SuperCapComm_t *this, const uint8_t rx_data[8], uint32_t rx_msg_std_id);

    void (*updateTxMsg)(SuperCapComm_t *this, SuperCap2ChassisMsg_t msg);

    Chassis2SuperCapMsg_t (*getRxMsg)(SuperCapComm_t *this);
    uint32_t kTxId;
    uint32_t kRxId;
    Chassis2SuperCapMsg_t chassis2cap_msg;
    SuperCap2ChassisMsg_t cap2chassis_msg;
  };

  extern SuperCapComm_t Cap_Com;
  void SuperCapCommInit(SuperCapComm_t *comm);
  extern void CAN_Send(void);
  extern uint32_t last_can_tick;
#ifdef __cplusplus
}
#endif
#endif

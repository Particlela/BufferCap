#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "hrtim.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "can.h"
#include "types.h"
#include "string.h"

SuperCapComm_t Cap_Com;
uint32_t last_can_tick = 0;

void can_filter_init(void);
static bool Decoder(SuperCapComm_t *this, const uint8_t rx_data[8], uint32_t rx_msg_std_id);
static void Encoder(SuperCapComm_t *this, uint8_t tx_data[8]);
static void UpdateTxMsg(SuperCapComm_t *this, SuperCap2ChassisMsg_t msg);
static Chassis2SuperCapMsg_t GetRxMsg(SuperCapComm_t *this);

void can_filter_init(void)//id过滤器注意配置
{
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x113;
  sFilterConfig.FilterID2 = 0x113;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ConfigInterruptLines(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, FDCAN_INTERRUPT_LINE0);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);
	HAL_FDCAN_Start(&hfdcan2);
}

void SuperCapCommInit(SuperCapComm_t *comm)
{
  comm->kTxId = 0x114; ///< 超电主控板发送给底盘信息
  comm->kRxId = 0x113; ///< 超电主控板接收底盘信息
  comm->decode = Decoder;
  comm->encode = Encoder;
  comm->updateTxMsg = UpdateTxMsg;
  comm->getRxMsg = GetRxMsg;
  comm->cap2chassis_msg.outpower  = 0;
  comm->cap2chassis_msg.remain_pct = 0;
  comm->cap2chassis_msg.volt_src = kJudge;
  comm->cap2chassis_msg.cap_volt = 0;
}

static bool Decoder(SuperCapComm_t *this, const uint8_t rx_data[8], uint32_t rx_msg_std_id)
{
  if (rx_msg_std_id != this->kRxId)
  {
    return false;
  }
  this->chassis2cap_msg.pwr_buf = (uint16_t)(rx_data[0] << 8 | rx_data[1]);
  this->chassis2cap_msg.pwr_limit = (uint16_t)(rx_data[2] << 8 | rx_data[3]);
  this->chassis2cap_msg.robot_hp = (uint16_t)(rx_data[4] << 8 | rx_data[5]);
  this->chassis2cap_msg.chassis_power = (uint16_t)(rx_data[6] << 8 | rx_data[7]) / 100.0f;

  return true;
}

static void Encoder(SuperCapComm_t *this, uint8_t tx_data[8])
{
  uint16_t remain_pct_tx = (uint16_t)(this->cap2chassis_msg.remain_pct * 100);
  uint16_t outpower_tx = (uint16_t)(this->cap2chassis_msg.outpower * 100);
  uint16_t cap_volt_tx = (uint16_t)(this->cap2chassis_msg.cap_volt * 100);
  tx_data[0] = (uint8_t)(remain_pct_tx >> 8);
  tx_data[1] = (uint8_t)(remain_pct_tx);
  tx_data[2] = (uint8_t)(this->cap2chassis_msg.volt_src);
  tx_data[3] = (uint8_t)(outpower_tx >> 8);
  tx_data[4] = (uint8_t)(outpower_tx);
  tx_data[5] = (uint8_t)(cap_volt_tx >> 8);
  tx_data[6] = (uint8_t)(cap_volt_tx);
}

static void UpdateTxMsg(SuperCapComm_t *this, SuperCap2ChassisMsg_t msg)
{
  memcpy(&(this->cap2chassis_msg), &msg, sizeof(SuperCap2ChassisMsg_t));
}

static Chassis2SuperCapMsg_t GetRxMsg(SuperCapComm_t *this)
{
  return this->chassis2cap_msg;
}

  FDCAN_RxHeaderTypeDef rx_header;
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  uint8_t rx_data[8];

  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);

  if (Cap_Com.decode(&Cap_Com, rx_data, rx_header.Identifier))
  {
    Control.buff_power = Cap_Com.chassis2cap_msg.pwr_buf;
    if (Cap_Com.chassis2cap_msg.pwr_limit > 130)
      Cap_Com.chassis2cap_msg.pwr_limit = 130;
    if (Cap_Com.chassis2cap_msg.pwr_limit < 10)
      Cap_Com.chassis2cap_msg.pwr_limit = 10;

		Control.power_set_target = Cap_Com.chassis2cap_msg.chassis_power;
		
    last_can_tick = HAL_GetTick();
  }
}

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef* hfdcan, uint32_t ErrorStatusITs)
{
	if (hfdcan == &hfdcan2 && (ErrorStatusITs & FDCAN_IT_BUS_OFF)) {
		FDCAN_ProtocolStatusTypeDef protocol_status;
		HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status);
		if(protocol_status.BusOff) {
			HAL_FDCAN_Init(hfdcan);
			can_filter_init();
		}
	}
}

uint8_t tx_data[8];

void CAN_Send(void)
{
  FDCAN_TxHeaderTypeDef tx_header;
  tx_header.Identifier = Cap_Com.kTxId;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;
  Cap_Com.encode(&Cap_Com, tx_data);
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, tx_data);
}

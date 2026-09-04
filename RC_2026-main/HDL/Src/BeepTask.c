 #include "BeepTask.h"
#include "CAN_IRQHandler.h"
#include "can.h"

#define BEEP_ON_MS  50
#define BEEP_OFF_MS 50

typedef enum
{
  BP_IDLE,
  BP_RUN,
  BP_REPLY
} BeepState;

static BeepState bp_state = BP_IDLE;

uint16_t beep_count_on = 0;
uint16_t beep_count_off = 0;
volatile uint8_t bp_done = 0;
volatile uint8_t bp_total = 0;

void BeepTask(void)
{
  switch (bp_state)
  {
  case BP_IDLE:
    if (g_beep_new)
    {
      g_beep_new = 0;
      bp_total = g_beep_count;
      bp_done = 0;

      if (bp_total == 0)
      {
        bp_state = BP_REPLY;
      }
      else
      {
        beep_count_on = BEEP_ON_MS;
        beep_count_off = 0;
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        bp_state = BP_RUN;
      }
    }
    break;

  case BP_RUN:
    if (bp_done >= bp_total)
    {
      bp_state = BP_REPLY;
    }
    break;

  case BP_REPLY:
  {
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[2] = {'O', 'K'};

    TxHeader.StdId = 0;
    TxHeader.ExtId = 0x02010101;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 2;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    bp_state = BP_IDLE;
    break;
  }

  default:
    break;
  }
}

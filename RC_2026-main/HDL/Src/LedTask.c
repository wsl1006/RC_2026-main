#include "LedTask.h"
#include "CAN_IRQHandler.h"
#include "can.h"

#define SHIFT_MS 100

typedef enum
{
  LED_OFF,
  LED_ON
} LedState;

static LedState led_state = LED_OFF;
static uint8_t led_phase = 0;
static uint32_t led_last[4];

static void led_on(uint16_t id)
{
  switch (id)
  {
  case 0:
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
    break;
  case 1:
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
    break;
  case 2:
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_SET);
    break;
  case 3:
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_SET);
    break;
  default:
    break;
  }
}

static void led_off(uint16_t id)
{
  switch (id)
  {
  case 0:
    HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
    break;
  case 1:
    HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
    break;
  case 2:
    HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);
    break;
  case 3:
    HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
    break;
  default:
    break;
  }
}

void LedTask(void)
{
  if (g_led_new)
  {
    g_led_new = 0;

    if (g_led_cmd == 0)
    {
      led_state = LED_OFF;
      for (uint8_t i = 0; i < 4; i++)
      {
        led_off(i);
      }
    }
    else
    {
      led_state = LED_ON;
      for (uint8_t i = 0; i < 4; i++)
      {
        led_off(i);
      }
      led_phase = 0;
      led_on(led_phase);
      led_last[led_phase] = HAL_GetTick();
    }

    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[3] = {'O', 'K', (uint8_t)led_state};

    TxHeader.StdId = 0;
    TxHeader.ExtId = 0x02010201;
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 3;
    TxHeader.TransmitGlobalTime = DISABLE;

    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
  }

  if (led_state == LED_ON &&
      HAL_GetTick() - led_last[led_phase] >= SHIFT_MS)
  {
    led_off(led_phase);
    led_phase = (led_phase + 1) % 4;
    led_last[led_phase] = HAL_GetTick();
    led_on(led_phase);
  }
}

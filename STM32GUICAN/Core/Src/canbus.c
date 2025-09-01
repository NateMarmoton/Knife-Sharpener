#include "canbus.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

FDCAN_TxHeaderTypeDef      ModeTxHeader;
FDCAN_TxHeaderTypeDef      SetPointTxHeader;

FDCAN_RxHeaderTypeDef      RxHeader;

uint8_t                    RxData[8];
uint8_t                    TxData[8];

extern TaskHandle_t        LvglTaskHandle;
extern TaskHandle_t        UI_TaskHandle;

extern FDCAN_HandleTypeDef hfdcan1;



/* RX Callback */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
	if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
	{
		/* Retrieve Rx messages from RX FIFO0 */
		if(HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
		{
			Error_Handler();
		}

		/* Display LEDx */
		if((RxHeader.Identifier == 0x98) && (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8))
		{
			int16_t MotorVelocity_int = (int16_t)((RxData[0] << 8) | RxData[1]);
			xTaskNotifyIndexedFromISR(LvglTaskHandle, MOTOR_VELOCITY, (uint32_t)MotorVelocity_int, eSetValueWithOverwrite, pdFALSE);
			xTaskNotifyIndexedFromISR(UI_TaskHandle, MOTOR_VELOCITY, (uint32_t)MotorVelocity_int, eSetValueWithOverwrite, pdFALSE);

			int16_t MotorCurrent_int = (int16_t)((RxData[2] << 8) | RxData[3]);
			xTaskNotifyIndexedFromISR(LvglTaskHandle, MOTOR_CURRENT, (uint32_t)MotorCurrent_int, eSetValueWithOverwrite, pdFALSE);
			xTaskNotifyIndexedFromISR(UI_TaskHandle, MOTOR_CURRENT, (uint32_t)MotorCurrent_int, eSetValueWithOverwrite, pdFALSE);

			int16_t MotorPosition_int = (int16_t)((RxData[4] << 8) | RxData[5]);
			xTaskNotifyIndexedFromISR(LvglTaskHandle, MOTOR_POSITION, (uint32_t)MotorPosition_int, eSetValueWithOverwrite, pdFALSE);

			int8_t Fault_int = RxData[6];
			xTaskNotifyIndexedFromISR(LvglTaskHandle, MOTOR_FAULTS, (uint32_t)Fault_int, eSetValueWithOverwrite, pdFALSE);

			int8_t Mode_int = RxData[7];
			xTaskNotifyIndexedFromISR(LvglTaskHandle, MOTOR_MODE, (uint32_t)Mode_int, eSetValueWithOverwrite, pdFALSE);
			xTaskNotifyIndexedFromISR(UI_TaskHandle, MOTOR_MODE, (uint32_t)Mode_int, eSetValueWithOverwrite, pdFALSE);
		}
	}
}

void FDCAN_Config(void)
{
	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType       = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex  = 0;
	sFilterConfig.FilterType   = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1    = 0x000;
	sFilterConfig.FilterID2    = 0x7FF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/* Start the FDCAN module */
	if(HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
	{
		Error_Handler();
	}

	/* Mode TX header */
	ModeTxHeader.Identifier          = 0x105;
	ModeTxHeader.IdType              = FDCAN_STANDARD_ID;
	ModeTxHeader.TxFrameType         = FDCAN_DATA_FRAME;
	ModeTxHeader.DataLength          = FDCAN_DLC_BYTES_8;
	ModeTxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	ModeTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
	ModeTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	ModeTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	ModeTxHeader.MessageMarker       = 0;


	/* SetPoint TX Header */
	SetPointTxHeader.Identifier          = 0x32;
	SetPointTxHeader.IdType              = FDCAN_STANDARD_ID;
	SetPointTxHeader.TxFrameType         = FDCAN_DATA_FRAME;
	SetPointTxHeader.DataLength          = FDCAN_DLC_BYTES_8;
	SetPointTxHeader.ErrorStateIndicator = FDCAN_ESI_PASSIVE;
	SetPointTxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
	SetPointTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	SetPointTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	SetPointTxHeader.MessageMarker       = 0;
}
/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:26:13
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-09 12:50:46
 * @FilePath: \MDK-ARMg:\project\stm32\f427iih6\RC\Core\Src\maincpp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "maincpp.h"
uint8_t common_buffer[8] = {0};
Motor::MotorInterface_t motor(1, &hcan1, common_buffer, true);
void can_filter_init(CAN_HandleTypeDef *_hcan);
int main_cpp()
{
		HAL_Delay(1000);
	
    can_filter_init(&hcan1);
    HAL_CAN_Start(&hcan1);
//    can_filter_init(&hcan2);
//    //   HAL_Delay(50);
//    HAL_CAN_Start(&hcan2);
    while (1)
    {
        motor.ControlOutput(1000);
        HAL_Delay(1);
    }
    return 0;
}

void can_filter_init(CAN_HandleTypeDef *_hcan)
{
    if (_hcan == &hcan2)
    {
        HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else
    {
        CAN_FilterTypeDef CAN_FilterConfigStructure;

        CAN_FilterConfigStructure.FilterBank = 0;
        CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;
        CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;
        CAN_FilterConfigStructure.FilterIdHigh = 0x0000;
        CAN_FilterConfigStructure.FilterIdLow = 0x0000;
        CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;
        CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;
        CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FilterFIFO0;
        CAN_FilterConfigStructure.SlaveStartFilterBank = 14;
        CAN_FilterConfigStructure.FilterActivation = ENABLE;

        if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
        {
            Error_Handler();
        }
			CAN_FilterConfigStructure.FilterBank = 14;
			if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK)
			{
					Error_Handler();
			}
       // HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}
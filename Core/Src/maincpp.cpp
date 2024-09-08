/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:26:13
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-08 21:56:01
 * @FilePath: \MDK-ARMg:\project\stm32\f427iih6\RC\Core\Src\maincpp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "maincpp.h"
uint8_t common_buffer[8] = {0};
Motor::MotorInterface_t motor(1, &hcan2, common_buffer, true);
int main_cpp()
{
    while (1)
    {
        motor.ControlOutput(1000);
        HAL_Delay(500);
    }
    return 0;
}
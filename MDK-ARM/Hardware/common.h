/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:53:31
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-13 23:50:33
 * @FilePath: \MDK-ARM\Hardware\common.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __COMMON_H
#define __COMMON_H
#define PI 3.1415926535
#include "main.h"
#include "stm32f4xx_hal.h"
#include "motor.h"
#include "arm_math.h"
#include "cmsis_os.h"
#include "kinematic.h"
extern SPI_HandleTypeDef hspi4;
extern UART_HandleTypeDef huart7;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
#endif
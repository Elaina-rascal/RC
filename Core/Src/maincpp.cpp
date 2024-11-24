/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:26:13
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-14 23:07:57
 * @FilePath: \MDK-ARMg:\project\stm32\f427iih6\RC\Core\Src\maincpp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "maincpp.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
uint8_t common_buffer[8] = {0};
//创建舵轮实例,其实应该用指针,但是这里为了方便直接用实例可能会导致外设初始化失败的问题
MotorModule_t module[4] = {
    {&hcan1, 3}, {&hcan1, 2}, {&hcan1, 1}, {&hcan1, 0}};
Kinematic::Kinematic_t kinematic;
TaskHandle_t Motor_Handle;
TaskHandle_t Kinematic_Handle;
TaskHandle_t Main_Handle;
/**
 * @brief can初始化 
 * 
 * @param _hcan 
 */
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan);
void Configure_Filter(void);
void Serial_Printf(char *format, ...);
int main_cpp()
{

    Configure_Filter();

    while (1)
    {
        kinematic.set_target(kinematic.target_val);
        for (int i = 0; i < 4; i++)
        {
            module[i].set_target(kinematic.Motor_speed_target[i], kinematic.Motor_angle_target[i],true);
        }
        HAL_Delay(10);
    }
    return 0;
}
/**
 * @description: 接收过滤器（全部接收）
 * @param {CAN_HandleTypeDef} *_hcan使用的CAN
 * @return {*}无
 */
void my_can_filter_init_recv_all(CAN_HandleTypeDef *_hcan)
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
        HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
}

void Configure_Filter(void)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; // 把接收到的报文放入到FIFO0
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterActivation = ENABLE;
    // sFilterConfig.SlaveStartFilterBank = 14;//为从CAN实例选择启动
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) // creat CanFilter
    {

        Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) // initialize can
    {

        Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
    }
    // 当FIFO0中有消息的时候进入中
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) // The FIFO0 receive interrupt function was enabled
    {

        Error_Handler(); //_Error_Handler(__FILE__, __LINE__);
    }
}
void Serial_Printf(char *format, ...)
{
    char String[100];              // 定义字符数组
    va_list arg;                   // 定义可变参数列表数据类型的变量arg
    va_start(arg, format);         // 从format开始，接收参数列表到arg变量
    vsprintf(String, format, arg); // 使用vsprintf打印格式化字符串和参数列表到字符数组中
    va_end(arg);                   // 结束变量arg
    // Serial_SendString(String);     // 串口发送字符数组（字符串）
    HAL_UART_Transmit(&huart7, (uint8_t *)String, strlen(String), 0xffff);
}
/**
 * @brief can接收中断
 * 
 * @param hcan 
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) // CAN接收中断
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
    case 0x201:
        // motor._angle_raw.data_int = (rx_data[0] << 8) | rx_data[1];
        // motor._vel_raw.data_int = (rx_data[2] << 8) | rx_data[3];
        break;
    default:
    {
        break;
    }
    }
}

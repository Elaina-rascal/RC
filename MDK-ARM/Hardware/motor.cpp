/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:56:31
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-12 23:42:03
 * @FilePath: \MDK-ARM\Hardware\motor.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "motor.h"
using namespace Motor;

void MotorInterface_t::ControlOutput(int16_t control)
{
    // int16_t remap_control = control / 2; // 映射

    // 第0个是电调1的高8位，第1个是电 调1的低8位，依次类推
    _common_buffer[_id % 4 - 1] = (control >> 8) & 0xFF;
    _common_buffer[_id % 4] = (control) & 0xFF;
    // 如果有发送can的权限，执行发送逻辑
    if (HaveTxPermission)
    {
        uint32_t TxMailbox;
        CAN_TxHeaderTypeDef Can_Tx;
        Can_Tx.DLC = 0x08;
        Can_Tx.ExtId = 0x0000;

        // 大疆的电机协议，1-4用0x200,5-8用0x1FF
        if (_id / 4 == 0)
        {
            Can_Tx.StdId = 0x200;
        }
        // Can_Tx.StdId = id;
        else
        {
            Can_Tx.StdId = 0x1FF;
        }
        Can_Tx.IDE = CAN_ID_STD;
        Can_Tx.RTR = CAN_RTR_DATA;
        Can_Tx.TransmitGlobalTime = DISABLE; // 不传输时间戳
        while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
        {
            /* code */
        }
        HAL_CAN_AddTxMessage(_hcan, &Can_Tx, _common_buffer, &TxMailbox);
    }
}
/**
 * @brief: 电机底层的更新函数，此处用不到为空,更新的逻辑在can中断
 * @return {*}
 * @note:
 */
void MotorInterface_t::update()
{
    switch (_type)
    {
    case Motor3508:
        break;
    case Motor2006:

        break;
    default:
        break;
    }
}
void Motor_t::set_speed_target(float target)
{
    _target = target * rev_fator * forward;
    pid.target_update(_target);
}

void Motor_t::ControlUpdate()
{
    update();
    // int16_t error = (_target + _rev_raw);
    int16_t control = pid.update(_rev_raw);
    ControlOutput(control);
}
/**
 * @brief 用SPI来获得电机的角度数据
 * @return {*}
 * @note:
 */
void Motor2006_Interface_t::angle_update(int16_t relative_angle)
{

    //     int16_t relative_angle_data = *((int16_t *)relative_angle);
}
void Motor2006_Interface_t::angle_update()
{
    uint16_t data;
    SPI_ReadWriteByte(0xFFFF);
    data = SPI_ReadWriteByte(0xFFFF);
    data &= 0x3FFF;
    //     if(data>=absolute_angle_zero)
    //     {
    //         absolute_angle_raw=data-absolute_angle_zero;
    //     }
    //     else
    //     {
    //         absolute_angle_raw=
    //     }
    (data >= absolute_angle_zero) ? absolute_angle_raw = data - absolute_angle_zero : absolute_angle_raw = data - absolute_angle_zero + absolute_angle_max;
}
uint16_t Motor2006_Interface_t::SPI_ReadWriteByte(uint16_t TxData)
{
    uint16_t rx_data;
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(_spi, (uint8_t *)&TxData, (uint8_t *)&rx_data, 1, 1000) != HAL_OK)
    {
        rx_data = 0;
    }
    HAL_GPIO_WritePin(_cs_port, _cs_pin, GPIO_PIN_SET);
    return rx_data;
}

void Motor2006_t::set_angle_target(float target)
{
    angle_target = target * angle_fator;
    angle_pid.target_update(angle_target);
}
void Motor2006_t::AngleControlUpdate()
{
    update();
    // 算出当前最近的角度
    int16_t angle_close = (_angle_raw - (int16_t)angle_target) > angle_raw_max / 2 ? _angle_raw - angle_raw_max : _angle_raw;
    int16_t control = angle_pid.update(angle_close);
    Motor_t::set_speed_target(control);

    Motor_t::ControlUpdate();
}
/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:56:31
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-13 23:20:41
 * @FilePath: \MDK-ARM\Hardware\motor.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "motor.h"
using namespace Motor;
#if USE_CAN_Motor
void MotorCanBase_t::CanSend(uint8_t *data, uint8_t len, uint32_t id)
{
    CAN_TxHeaderTypeDef Can_Tx;
    Can_Tx.DLC = len;
    Can_Tx.ExtId = 0x0000;
    Can_Tx.StdId = id;
    Can_Tx.IDE = CAN_ID_STD;
    Can_Tx.RTR = CAN_RTR_DATA;
    Can_Tx.TransmitGlobalTime = DISABLE; // 不传输时间戳
    uint32_t TxMailbox;
    while (HAL_CAN_GetTxMailboxesFreeLevel(_can) == 0)
    {
        /* code */
    }
    HAL_CAN_AddTxMessage(_can, &Can_Tx, data, &TxMailbox);
}
#if USE_3508
void Motor3508_t::set_speed_target(float target)
{
    _vel_target = target * rev_fator * forward;
    pid.target_update(_vel_target);
}
void Motor3508_t::update()
{
    int16_t control = pid.update(_vel_raw.data_int);
    _common_buffer[_id % 4 - 1] = (control >> 8) & 0xFF;
    _common_buffer[_id % 4] = (control) & 0xFF;
    if (have_tx_permission)
    {
        uint32_t Canid = 0x200;
        if (_id / 4 == 1)
        {
            Canid = 0x1FF;
        }
        CanSend(_common_buffer, 8, Canid);
    }
}
#endif
#if USE_SteeringWheelModel
// void MotorModule_t::set_target(int16_t vel_target, int16_t angle_target)
// {
//     _vel_target_union.data_int = vel_target;
//     _angle_target_union.data_int = angle_target;
//     uint8_t data[8];
//     data[0] = ((int16_t)_angle_target_union.data_int) >> 8;
//     data[1] = (int16_t)_angle_target_union.data_int;
//     data[2] = ((int16_t)_vel_target_union.data_int) >> 8;
//     data[3] = (int16_t)_vel_target_union.data_int;
//     CanSend(data, 8, _id);
// }
void MotorModule_t::set_target(float vel_target, float angle_target)
{
    _vel_target = vel_target;
    _angle_target = angle_target;
    vel_int = vel_target * vel_factor*forward;
    angle_int = angle_target * angle_factor;
    uint8_t data[8];
    data[0] = ((int16_t)angle_int) >> 8;
    data[1] = (int16_t)angle_int;
    data[2] = ((int16_t)vel_int) >> 8;
    data[3] = (int16_t)vel_int;
    CanSend(data, 8, _id);
}
#endif
#endif
#if USE_CAN_AbsoluteMotor
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
#endif

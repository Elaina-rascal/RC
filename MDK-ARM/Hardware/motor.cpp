/**
 * @file motor.cpp
 * @author Elaina (1463967532@qq.com)
 * @brief 这是电机的驱动文件,包括舵轮分控与C620电调
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "motor.h"
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
    _common_buffer[(_id - 1) % 4 * 2] = (control >> 8) & 0xFF;
    _common_buffer[(_id - 1) % 4 * 2] = (control) & 0xFF;
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
float MotorModule_t::normalize_angle(float angle)
{
    while (angle > PI)
    {
        angle -= 2 * PI;
    }
    while (angle < -PI)
    {
        angle += 2 * PI;
    }
    return angle;
}
void MotorModule_t::set_target(float vel_target, float angle_target, bool use_youhua)
{
    _vel_target = vel_target;
    _angle_target = angle_target - angle_zero;

    if (use_youhua)
    {
        // 算出轮子不反向角度与轮子反向角度
        float delta_theta_forward = normalize_angle(_angle_target - angle_last);
        float delta_theta_reverse = normalize_angle(normalize_angle(_angle_target + PI) - angle_last);
        if (fabs(delta_theta_forward) > fabs(delta_theta_reverse))
        {
            _vel_target = -_vel_target;
            _angle_target = normalize_angle(_angle_target + PI);
        }
        else
        {

            _angle_target = normalize_angle(_angle_target);
        }
    }

    vel_int = _vel_target * vel_factor * forward;
    angle_int = _angle_target * angle_factor;

    uint8_t data[8];
    data[0] = ((int16_t)angle_int) >> 8;
    data[1] = (int16_t)angle_int;
    data[2] = ((int16_t)vel_int) >> 8;
    data[3] = (int16_t)vel_int;
    CanSend(data, 8, _id);
    angle_last = _angle_target;
}
#endif
#endif


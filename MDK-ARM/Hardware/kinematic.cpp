/*
 * @Author: Elaina
 * @Date: 2024-09-11 17:49:32
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-11 18:03:12
 * @FilePath: \MDK-ARM\Hardware\kinematic.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "kinematic.h"
using namespace Kinematic;

/**
 * @brief 根据自身坐标系从底盘速度解算电机速度
 * @return {*}
 * @note:
 */
void Kinematic_t::inv(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle)
{
}

/**
 * @brief 从电机速度解算底盘速度
 * @return {*}
 * @note:
 */
void Kinematic_t::forward(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle)
{
}

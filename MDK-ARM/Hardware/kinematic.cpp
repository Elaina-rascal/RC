/**
 * @file kinematic.cpp
 * @author Elaina (1463967532@qq.com)
 * @brief 舵轮解算软件库 
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "kinematic.h"
#define MY_SIN_CNT(x) arm_sin_f32(x)
#define MY_COS_CNT(x) arm_cos_f32(x)
#define MY_SQRT_CNT(x) arm_sqrt_f32(x)
using namespace Kinematic;
/**
 * @brief 设置目标值
 * 
 * @param cmd_vel_in   目标速度,局部坐标系 
 */
void Kinematic_t::set_target(cmd_vel_t cmd_vel_in)
{
    target_val = cmd_vel_in;
    inv(&target_val, Motor_speed_target, Motor_angle_target);
}
/**
 * @brief 根据自身坐标系从底盘速度解算电机速度
 * 
 * @param cmd_vel_in 目标速度
 * @param Motor_speed 解算出的速度,传入float4
 * @param Motor_angle 解算出的角度,传入float4
 */
void Kinematic_t::inv(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle)
{
    float linear_x = cmd_vel_in->linear_x;
    float linear_y = cmd_vel_in->linear_y;
    float angular_z = cmd_vel_in->angular_z;
    float vel_total = sqrt((linear_x - angular_z * L / 2) * (linear_x - angular_z * L / 2) + (linear_y + angular_z * W / 2) * (linear_y + angular_z * W / 2));
    float vel_total2 = sqrt((linear_x + angular_z * L / 2) * (linear_x + angular_z * L / 2) + (linear_y + angular_z * W / 2) * (linear_y + angular_z * W / 2));
    if (vel_total > 0.05)
    {
        Motor_angle[0] = atan2(linear_y + angular_z * W / 2, linear_x - angular_z * L / 2);
        Motor_angle[1] = atan2(linear_y + angular_z * W / 2, linear_x + angular_z * L / 2);
        Motor_angle[2] = atan2(linear_y - angular_z * W / 2, linear_x - angular_z * L / 2);
        Motor_angle[3] = atan2(linear_y - angular_z * W / 2, linear_x + angular_z * L / 2);
    }
    Motor_speed[0] = vel_total;
    Motor_speed[1] = vel_total2;
    Motor_speed[2] = vel_total;
    Motor_speed[3] = vel_total2;
}
void Kinematic_t::update()
{
    switch (mode)
    {
    case OpenControl:

        break;

    default:
        break;
    }
    // inv(&target_val, Motor_speed_target, Motor_angle_target);
    // forward(&current_vel, Motor_speed_current, Motor_angle_current);
}
/**
 * @brief 从电机速度解算底盘速度
 * @return {*}
 * @note:
 */
void Kinematic_t::forward(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle)
{
}
/**
 * @brief 角度归一化到-PI到PI
 * 
 * @param angle 
 * @return float 
 */
float Kinematic::normalize_angle(float angle)
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

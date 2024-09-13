/*
 * @Author: Elaina
 * @Date: 2024-09-11 17:49:32
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-11 18:06:15
 * @FilePath: \MDK-ARM\Hardware\kinematic.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include "common.h"
namespace Kinematic
{

    struct cmd_vel_t
    {
        float linear_x;
        float linear_y;
        float angular_z;
    };
    enum Kinematic_mode_t
    {
        OpenControl,
    };
    enum Kinematic_type_t
    {
        four_wheel_steering
    };
    class Kinematic_t
    {
    public:
        Kinematic_t()
        {
        }
        void inv(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle);
        void forward(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle);
        void update();
        cmd_vel_t current_vel;
        cmd_vel_t target_val;
        float Motor_speed_target[4];
        float Motor_angle_target[4];
        float Motor_speed_current[4];
        float Motor_angle_current[4];

    private:
        Kinematic_mode_t mode = OpenControl;
        Kinematic_type_t type = four_wheel_steering;
        float L; // 前后轮之间的距离
        float W; // 左右轮之间的距离
    };

} // namespace Kinematic
#endif
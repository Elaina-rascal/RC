/**
 * @file kinematic.h
 * @author Elaina (1463967532@qq.com)
 * @brief 舵轮结算头文件
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __KINEMATIC_H
#define __KINEMATIC_H
#include "common.h"
namespace Kinematic
{

    /*右后为3号，左后为2号，右前为1号，左前为0号*/
    struct odom_t
    {
        double x;
        double y;
        double yaw;
    };
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

    class Kinematic_t
    {
    public:
        Kinematic_t()
        {
        }
        void set_target(cmd_vel_t cmd_vel_in);
        void inv(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle);
        void forward(cmd_vel_t *cmd_vel_in, float *Motor_speed, float *Motor_angle);
        void update();
        cmd_vel_t current_vel;
        cmd_vel_t target_val;
        float Motor_speed_target[4];
        float Motor_angle_target[4];
        float Motor_speed_current[4];
        float Motor_angle_current[4];
        float Motor_angle_last[4];

    private:
        Kinematic_mode_t mode = OpenControl;

        float L = 0.425; // 前后轮之间的距离
        float W = 0.425; // 左右轮之间的距离
    };
    float normalize_angle(float angle);
} // namespace Kinematic
#endif
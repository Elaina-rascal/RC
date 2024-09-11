/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:56:31
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-11 17:41:44
 * @FilePath: \MDK-ARM\Hardware\motor.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __MOTOR_H
#define __MOTOR_H
#include "common.h"
#include "pid_template.h"
namespace Motor
{
    enum MotorType_t
    {
        Steering_Motor,
        Drive_Motor,
        Motor3508,
        Motor2006

    };
    // 接口类
    class MotorInterface_t
    {
    public:
        MotorInterface_t()
        {
        }
        void bind_pin(uint8_t id, CAN_HandleTypeDef *hcan, uint8_t *common_buffer, bool HavePermission = false)
        {
            _id = id;
            _hcan = hcan;
            _common_buffer = common_buffer;
            HaveTxPermission = HavePermission;
        }

        /*提供给上层的接口部分*/
        void ControlOutput(int16_t control);
        void update();
        int16_t _rev_raw;   // 转速原始数据
        int16_t _angle_raw; // 角度原始数据

        MotorType_t _type = Motor3508;

    protected:
        /*因为一般原始数据要大于真实数据，所以这里取真实数据到原始数据的映射*/
        float rev_fator = 33;       // 转速系数 从原始数据转化到真实数据的倒数
        float angle_fator = 22.475; // 角度系数从真实数据到原始数据
        int16_t angle_raw_max = 8192;

    private:
        uint8_t *_common_buffer;
        uint8_t _id;
        CAN_HandleTypeDef *_hcan;
        bool HaveTxPermission = false;
    };

    class Motor_t : public MotorInterface_t
    {
    public:
        Motor_t()
        {
        }
        void set_speed_target(float target);
        void ControlUpdate();
        float debug;     // 给调试用的
        int forward = 1; // 正反转
    protected:
        float _target;
        pid_base_template_t<int, float> pid = pid_base_template_t<int, float>({5, 2, 0, -5000, 5000, 2000});
    };
    class Motor2006_t : public Motor_t
    {
    public:
        Motor2006_t()
        {
            pid.pid_update(5, 1.4, 0.5); // 6002 pid参数
            _type = Motor2006;
        }
        void set_angle_target(float target);
        void AngleControlUpdate();

    private:
        float angle_target;
        pid_base_template_t<int, float> angle_pid = pid_base_template_t<int, float>({0.1, 0, 0, -60, 60, 2000});
    };
}
#endif

/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:56:31
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-12 21:49:35
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
        pid_base_template_t<int16_t, float> pid = pid_base_template_t<int16_t, float>({5, 2, 0, -5000, 5000, 2000});
    };

    /*与2006绝对式编码器相关部分*/
    // 2006的基础派生类
    /*与绝对编码相关的部分*/
    class Motor2006_Interface_t
    {
    public:
        Motor2006_Interface_t()
        {
        }
        Motor2006_Interface_t(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, int16_t zero_data)
        {
            _spi = spi;
            _cs_port = cs_port;
            _cs_pin = cs_pin;
            absolute_angle_zero = zero_data;
        }
        void angle_update(int16_t relative_angle);
        void angle_update();

    protected:
        int16_t absolute_angle_max = 16383; // 绝对编码器的最大值
        int16_t absolute_angle_raw;         // 绝对编码器的原始值
        int16_t absolute_angle_zero;        // 绝对式编码器原点

        int16_t Turns_num;                 // 转动的圈数
        int16_t Last_relative_angle;       // 上一次的相对角度
        int16_t relative_angle_max = 8192; // 相对编码器的最大值
        float Turns_factor;                // 从圈数转化到角度的系数
        float real_angle;                  // 最后的当前真实角度

    private:
        uint16_t SPI_ReadWriteByte(uint16_t TxData);
        SPI_HandleTypeDef *_spi;
        GPIO_TypeDef *_cs_port;
        uint16_t _cs_pin;
    };
    class Motor2006_t : public Motor_t, public Motor2006_Interface_t
    {
    public:
        Motor2006_t(SPI_HandleTypeDef *spi, GPIO_TypeDef *cs_port, uint16_t cs_pin, int16_t zero_data) : Motor2006_Interface_t(spi, cs_port, cs_pin, zero_data)
        {
            Motor2006_t();
        }
        Motor2006_t()
        {
            _type = Motor2006;
            pid.pid_update(5, 1.4, 0.5); // 6002 pid参数
        }
        void set_angle_target(float target);
        void AngleControlUpdate();

    private:
        float zero_angle;
        float angle_target;
        pid_base_template_t<int16_t, float> angle_pid = pid_base_template_t<int16_t, float>({0.1, 0, 0, -60, 60, 2000});
    };
}
#endif

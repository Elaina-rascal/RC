/*
 * @Author: Elaina
 * @Date: 2024-09-08 14:56:31
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-09-13 23:20:27
 * @FilePath: \MDK-ARM\Hardware\motor.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __MOTOR_H
#define __MOTOR_H
#define USE_CAN_Motor 1 // 使用CAN电机
#define USE_3508 1
#define USE_CAN_AbsoluteMotor 0  // 使用CAN绝对编码电机
#define USE_SteeringWheelModel 1 // 使用舵轮模块
#define USE_Debug 1              // 使用调试
#include "common.h"
#include "pid_template.h"
namespace Motor
{
    union data_t
    {
        uint8_t data_raw[4];
        float data_float;
        int32_t data_int;
        // int16_t data_int16;
    };

    enum MotorType_t
    {
        Steering_Motor,
        Drive_Motor,
        Motor3508,
        Motor2006

    };
    class MotorBase_t
    {
    public:
        MotorBase_t()
        {
        }
        uint8_t _id;
        float _vel_target;
        float _angle_target;
        // int16_t _vel_raw;   // 转速原始数据
        // int16_t _angle_raw; // 角度原始数据
        data_t _vel_raw;
        data_t _angle_raw;
        // data_t _vel_target_union;
        // data_t _angle_target_union;
        float vel_target;
        float angle_target;
#if USE_Debug
        float debug; // 给调试用的
#endif
    };
#if USE_CAN_Motor
    // 接口类
    class MotorCanBase_t : public MotorBase_t
    {
    public:
        MotorCanBase_t()
        {
        }
        void bind_pin(CAN_HandleTypeDef *hcan, uint8_t id)
        {
            _can = hcan;
            _id = id;
        }

    protected:
        void CanSend(uint8_t *data, uint8_t len, uint32_t id);

    private:
        CAN_HandleTypeDef *_can;
        // uint8_t id;
    };
#if USE_3508
    class Motor3508_t : public MotorCanBase_t
    {
    public:
        Motor3508_t(uint8_t id, bool have_tx_permission = false) : MotorCanBase_t()
        {
            _id = id;
            this->have_tx_permission = have_tx_permission;
        }
        void set_speed_target(float target);
        void update();
        // float debug;
        int forward = 1; // 正反转
    private:
        uint8_t _common_buffer[8];

        float rev_fator = 33; // 转速系数 从原始数据转化到真实数据的倒数
        bool have_tx_permission = false;

        pid_base_template_t<int16_t, float> pid = pid_base_template_t<int16_t, float>({5, 2, 0, -5000, 5000, 2000});
    };

#endif
#if USE_SteeringWheelModel
    /*封装成模块的舵轮分块*/
    class MotorModule_t : public MotorCanBase_t
    {
    public:
        // void set_target(int16_t vel_target, int16_t angle_target);
        void set_target(float vel_target, float angle_target);

    private:
        int8_t forward=1;
        int16_t vel_int;
        int16_t angle_int;
        int16_t angle_factor = 2608; //45.51/PI*180
        int16_t vel_factor = 2000;
    };
}
#endif
#endif
/*与2006绝对式编码器相关部分*/
// 2006的基础派生类
/*与绝对编码相关的部分没写完*/
#if USE_CAN_AbsoluteMotor
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
#endif

#endif

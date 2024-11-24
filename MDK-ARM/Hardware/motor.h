/**
 * @file motor.h
 * @author Elaina (1463967532@qq.com)
 * @brief 这是电机的驱动文件,包括舵轮分控与C620电调
 * @version 0.1
 * @date 2024-11-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef __MOTOR_H
#define __MOTOR_H
#define USE_CAN_Motor 1 // 使用CAN电机
#define USE_3508 1
#define USE_SteeringWheelModel 1 // 使用舵轮模块
#define USE_Debug 1              // 使用调试
#include "common.h"
#include "pid_template.h"
    /**
     * @brief 电机的原始数据联合体
     * 
     */
    union data_t
    {
        uint8_t data_raw[4];
        float data_float;
        int32_t data_int;
        // int16_t data_int16;
    };

    /**
     * @brief 电机接口类
     * 
     */
    class IMotorSpeed_t
    {
    public:
        /**
         * @brief 设置目标线速度
         * 
         * @param target 
         */
        virtual void set_speed_target(float target) = 0;
        /**
         * @brief 电机更新
         * 
         */
        virtual void update() = 0;
        /**
         * @brief 设置目标角度
         * 
         * @param target 
         */
        virtual void set_angle_target(float target)
        {
        }
    };
    /**
     * @brief 电机基础类
     * 
     */
    class MotorBase_t
    {
    public:
        MotorBase_t()
        {
        }
        MotorBase_t(uint8_t id) : _id(id)
        {
        }
        uint8_t _id=0;        
        float _vel_target=0;      //目标线速度
        float _angle_target=0;    //目标角度
        data_t _vel_raw; // 转速原始数据
        data_t _angle_raw; // 角度原始数据
        float _radium=0;     //电机半径
#if USE_Debug
        float debug=0; // 给调试用的
#endif
    };
#if USE_CAN_Motor
    /**
     * @brief CAN的基础封装类,包含发送can的函数与绑定can的函数
     * 
     */
    class MotorCanBase_t : public MotorBase_t
    {
    public:
        MotorCanBase_t()
        {
        }
        MotorCanBase_t(CAN_HandleTypeDef *hcan, uint8_t id) : MotorBase_t(id), _can(hcan)
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
    /**
     * @brief c620电调电机类,默认数据适配3508
     * 
     */
    class Motor3508_t : public MotorCanBase_t, public IMotorSpeed_t
    {
    public:
        /**
         * @brief Construct a new Motor3508_t object
         * 
         * @param id 电机的id
         * @param have_tx_permission //电机的发送权限 
         */
        Motor3508_t(uint8_t id, bool have_tx_permission = false) : MotorCanBase_t()
        {
            _id = id;
            this->have_tx_permission = have_tx_permission;
        }
        void set_speed_target(float target) override;
        void setLinearSpeed(float linear_speed);
        void update() override;
        // float debug;
        int forward = 1; // 正反转
    private:
        uint8_t _common_buffer[8];

        float rev_fator = 33; // 轴转速转速系数,真实数据=原始数据*rev_fator
        bool have_tx_permission = false;

        pid_base_template_t<int16_t, float> pid = pid_base_template_t<int16_t, float>({5, 2, 0, -5000, 5000, 2000});
    };

#endif
#if USE_SteeringWheelModel
    /**
     * @brief 封装成模块的电机类,适配2024年电机主控板
     * 
     */
    class MotorModule_t : public MotorCanBase_t
    {
    public:
        MotorModule_t()
        {
        }
        MotorModule_t(CAN_HandleTypeDef *hcan, uint8_t id) : MotorCanBase_t(hcan, id)
        {
            // _type = Steering_Motor;
        }
        void set_target(float vel_target, float angle_target, bool use_youhua = false);
        static float normalize_angle(float angle);

    private:
        float angle_last;  //上次的距离,给舵轮优化用
        int8_t forward = 1;
        int16_t angle_factor = 2608; // 45.51/PI*180
        int16_t vel_factor = 2000;
        float angle_zero = PI / 2;
    };

#endif
#endif

#endif

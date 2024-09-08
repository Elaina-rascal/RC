#ifndef __MOTOR_H
#define __MOTOR_H
#include "common.h"
#include "pid_template.h"
namespace Motor
{
    enum MotorType_t
    {
        Steering_Motor,
        Drive_Motor
    };
    // 接口类
    class MotorInterface_t
    {
    public:
        void bind_pin();
        MotorInterface_t(uint8_t id, CAN_HandleTypeDef *hcan, uint8_t *common_buffer, bool HavePermission = false) : _id(id),
                                                                                                                     _hcan(hcan), _common_buffer(common_buffer), HaveTxPermission(HavePermission)
        {
            HAL_CAN_Start(_hcan);
        };
        void ControlOutput(int16_t control);

    protected:
        MotorType_t _type = Drive_Motor;

    private:
        uint8_t *_common_buffer;
        uint8_t _id;
        CAN_HandleTypeDef *_hcan;
        bool HaveTxPermission = false;
    };

    class Motor_t : public MotorInterface_t
    {
    public:
        void set_target();
        void ControlUpdate();

    private:
        float target;
        pid_base_template_t<float> pid = {};
    };
}
#endif

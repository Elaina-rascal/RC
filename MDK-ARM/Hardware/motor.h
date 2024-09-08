#ifndef __MOTOR_H
#define __MOTOR_H
#include "common.h"
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
        void ControlOutput(float control);

    protected:
        MotorType_t type = Drive_Motor;
    };

    class Motor_t : public MotorInterface_t
    {
    public:
        void target_update();

    private:
        float target;
    };
}
#endif

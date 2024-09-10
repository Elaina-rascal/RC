#include "motor.h"
using namespace Motor;

void MotorInterface_t::bind_pin()
{
}
void MotorInterface_t::ControlOutput(int16_t control)
{

    // 第0个是电调1的高8位，第1个是电 调1的低8位，依次类推
    _common_buffer[_id % 4 - 1] = (control >> 8) & 0xFF;
    _common_buffer[_id % 4] = (control) & 0xFF;
    if (HaveTxPermission)
    {
        uint32_t TxMailbox;
        CAN_TxHeaderTypeDef Can_Tx;
        Can_Tx.DLC = 0x08;
        Can_Tx.ExtId = 0x0000;

        // 大疆的电机协议，1-4用0x200,5-8用0x1FF
        if (_id / 4 == 0)
        {
            Can_Tx.StdId = 0x200;
        }
        // Can_Tx.StdId = id;
        else
        {
            Can_Tx.StdId = 0x1FF;
        }
        Can_Tx.IDE = CAN_ID_STD;
        Can_Tx.RTR = CAN_RTR_DATA;
        Can_Tx.TransmitGlobalTime = DISABLE; // 不传输时间戳
        // while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
        // {
        //     /* code */
        // }
        HAL_CAN_AddTxMessage(_hcan, &Can_Tx, _common_buffer, &TxMailbox);
    }
}

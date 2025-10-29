#pragma once

#include "cmsis_os.h"
#include "main.h"
#include "Referee.hpp"

extern Referee referee;

class BoardCommunicate {
private:
    CAN_HandleTypeDef *hcan;

public:
    uint8_t RawData[16][8];

    enum ChassisRunType {
        Torque2Zero = 0,
        Speed2Zero,
        Normal
    } chassis_run_type;

    enum ChassisFollowType {
        Follow = 0,
        Counterclockwise,
        Clockwise
    };

    enum ChassisType
    {
        TorqueZero_type = 0,
        SpeedZero_type,
        Follow_type,
        F_Gyro_type,
        Gyro_type
    }chassis_type;

    int16_t chassis_target_vx, chassis_target_vy;

    BoardCommunicate(CAN_HandleTypeDef *hcan) : hcan(hcan) {
    }

    void Send(ChassisRunType run_type, int16_t vx, int16_t vy, ChassisFollowType chassis_follow_type) {
        this->chassis_run_type = run_type;
        this->chassis_target_vx = vx;
        this->chassis_target_vy = vy;
        RawData[0][0] = run_type;
        RawData[0][1] = vx >> 8;
        RawData[0][2] = vx;
        RawData[0][3] = vy >> 8;
        RawData[0][4] = vy;
        RawData[0][5] = chassis_follow_type;
        Send(0);
    }

    void Send_UI(uint8_t key_g,uint8_t FriWheel_status, uint8_t Circle_status, uint8_t key_shift, uint8_t can_shoot)
    {
        RawData[3][0] = key_g;
        RawData[3][1] = FriWheel_status;
        RawData[3][2] = Circle_status;
        RawData[3][3] = key_shift;
        RawData[3][4] = can_shoot;
        Send(3);
    }


    void Send(uint8_t send_idx) {
        CAN_TxHeaderTypeDef canTxHeader;
        uint32_t send_mail;
        canTxHeader.DLC = 0x08;
        canTxHeader.RTR = CAN_RTR_DATA;
        canTxHeader.IDE = CAN_ID_STD;
        canTxHeader.StdId = 0x100 + send_idx;
        HAL_CAN_AddTxMessage(hcan, &canTxHeader, RawData[send_idx], &send_mail);
        osDelay(1);
    }

};

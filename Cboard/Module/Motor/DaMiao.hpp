#ifndef DAMIAO_H
#define DAMIAO_H

#include "main.h"
#include "Motor.hpp"
#include "PID.hpp"

#define VAL_LIMIT(val, min, max) if(val<=min){val = min;}else if(val>=max){val = max;}

//十六进制到浮点数
static float Hex_To_Float(const uint32_t *Byte, int num) {
    return *((float *) Byte);
}

//浮点数到十六进制转换
static uint32_t Float_To_hex(float HEX) {
    return *(uint32_t *) &HEX;
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x_float - offset) * ((float) ((1 << bits) - 1)) / span);
}

/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float) x_int) * span / ((float) ((1 << bits) - 1)) + offset;
}

class DaMiao : public Motor {
private:

    CAN_HandleTypeDef *hcan;
    CAN_TxHeaderTypeDef canTxHeader;
    volatile uint32_t send_mail;
    float P_MIN = -3.141593f;
    float P_MAX = 3.141593f;
    float V_MIN = -30.0f;
    float V_MAX = 30.0f;
    float KP_MIN = 0.0f;
    float KP_MAX = 500.0f;
    float KD_MIN = 0.0f;
    float KD_MAX = 5.0f;
    float T_MIN = -10.0f;
    float T_MAX = 10.0f;

public:
    PID speed_pid, position_pid;
    uint8_t raw_can_data[8];
    uint16_t sendID, receiveID;
    uint16_t state;
    int p_int;
    int v_int;
    int t_int;
    int kp_int;
    int kd_int;
    float pos;
    float vel;
    float tor;
    float Kp;
    float Kd;
    float Tmos;
    float Tcoil;
    float target_Tcoil;
    float set_tor;

    enum CtrlMode {
        MIT = 0,
        POSITION = 1,
        SPEED = 2
    };

    DaMiao(CAN_HandleTypeDef *hcan, uint32_t sendID, uint32_t receiveID, enum CtrlMode ctrl_mode) : sendID(sendID),
                                                                                                    receiveID(
                                                                                                            receiveID),
                                                                                                    hcan(hcan) {
        canTxHeader.DLC = 0x08;
        canTxHeader.RTR = CAN_RTR_DATA;
        canTxHeader.IDE = CAN_ID_STD;
        switch (ctrl_mode) {
            case MIT:
                canTxHeader.StdId = sendID + 0x000;
                break;
            case POSITION:
                canTxHeader.StdId = sendID + 0x100;
                break;
            case SPEED:
                canTxHeader.StdId = sendID + 0x200;
                break;
        }
    }

    void Update() override;

    void EnableMotor();

    void DisableMotor();

    void MIT_Control(float pos, float vel, float kp, float kd, float torq);

    void setSpeedExpect(float ref, float fdk);

    void setSpeedExpect(float ref);

    void PositionSpeedControl(float pos, float vel);

    void SpeedControl(float vel);
};

#endif //DAMIAO_H

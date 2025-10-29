#include "DJMotor.hpp"
#include "can.h"

//  can1/can2   0x200/0x1FF/0x1FE/0x2FE data
uint8_t canTx[2][4][8] = {0};

void ControlDJMotor() {
    CAN_TxHeaderTypeDef canTxHeader;
    uint32_t send_mail;
    canTxHeader.DLC = 0x08;
    canTxHeader.RTR = CAN_RTR_DATA;
    canTxHeader.IDE = CAN_ID_STD;
    // canTxHeader.StdId = 0x200;
    // HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, canTx[0][0], &send_mail);
    // HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTx[1][0], &send_mail);
    canTxHeader.StdId = 0x1FF;//两个3508
    HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, canTx[0][1], &send_mail);
    // HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTx[1][1], &send_mail);
    // canTxHeader.StdId = 0x1FE;
    // HAL_CAN_AddTxMessage(&hcan1, &canTxHeader, canTx[0][2], &send_mail);
    // HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTx[1][2], &send_mail);
    canTxHeader.StdId = 0x200;
    HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTx[1][0], &send_mail);
    // HAL_CAN_AddTxMessage(&hcan2, &canTxHeader, canTx[1][3], &send_mail);
}

void DJMotor::Update() {
    last_rotor_raw_angle = rotor_raw_angle;
    rotor_raw_angle = static_cast<int16_t>(raw_can_data[0] << 8 | raw_can_data[1]);
    rotor_rate = static_cast<int16_t>(raw_can_data[2] << 8 | raw_can_data[3]);
    electric_current = static_cast<int16_t>(raw_can_data[4] << 8 | raw_can_data[5]);
    temperature = raw_can_data[6];
    rotor_angle = static_cast<float>(rotor_raw_angle) / 8192.0f * 360;//角度制

    if (rotor_raw_angle - last_rotor_raw_angle < -4096) {
        rotor_circle_number++;
    } else if (rotor_raw_angle - last_rotor_raw_angle > 4096) {
        rotor_circle_number--;
    }
    rotor_accumlate_angle = static_cast<float>(rotor_circle_number) * 360.0f + rotor_angle;
    rate = rotor_rate / reduction_ratio;
    accumlate_angle = static_cast<float>(rotor_circle_number * 8192 + rotor_raw_angle) /
                      reduction_ratio / 8192.0f * 360;//是输出轴的累计角度
    switch (motor_type) {
        case M3508:
            electric_current = electric_current / 16384 * 20;
            break;
        case M2006:
            electric_current = electric_current / 10000 * 10;
            break;
        case GM6020:
            electric_current = electric_current / 16384 * 3;
            break;
        default:
            break;
    }
    connection_lost_count++;
    if (connection_lost_count > 100) {
        connection_lost = true;
    }
}
//这两个分别用来设置反馈值为电机原始参数和不是电机原始参数
void DJMotor::setSpeedExpect(float ref) {
    target_rate = ref;
    Update();
    isLockedRotor = speed_pid.calc(target_rate, rate);
    PopulateControlPackets();
}

void DJMotor::setSpeedExpect(float ref, float fdk) {
    target_rate = ref;
    Update();
    isLockedRotor = speed_pid.calc(ref, fdk);
    PopulateControlPackets();
}

void DJMotor::PopulateControlPackets() const {
    if (motor_type == GM6020) {
        if (feedback_id < 0x209) {
            canTx[which_can][2][(feedback_id - 0x205) * 2] = (int) speed_pid.output >> 8;
            canTx[which_can][2][(feedback_id - 0x205) * 2 + 1] = (int) speed_pid.output;
        } else {
            canTx[which_can][3][(feedback_id - 0x209) * 2] = (int) speed_pid.output >> 8;
            canTx[which_can][3][(feedback_id - 0x209) * 2 + 1] = (int) speed_pid.output;
        }
    } else {
        if (feedback_id < 0x205) {
            canTx[which_can][0][(feedback_id - 0x201) * 2] = (int) speed_pid.output >> 8;
            canTx[which_can][0][(feedback_id - 0x201) * 2 + 1] = (int) speed_pid.output;
        } else {
            canTx[which_can][1][(feedback_id - 0x205) * 2] = (int) speed_pid.output >> 8;
            canTx[which_can][1][(feedback_id - 0x205) * 2 + 1] = (int) speed_pid.output;
        }
    }
}

void DJMotor::ZeroTorqueStop() {
    speed_pid.output = 0;
    PopulateControlPackets();
}

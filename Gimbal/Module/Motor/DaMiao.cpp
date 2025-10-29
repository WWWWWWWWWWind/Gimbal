#include "DaMiao.hpp"
// uint16_t text;

/**
************************************************************************
* @brief:      	获取DM4310电机反馈数据函数
* @param[in]:   void
* @retval:     	void
* @details:    	从接收到的数据中提取DM4310电机的反馈信息，包括
*               电机ID、状态、位置、速度、扭矩相关温度参数、寄存器数据等
************************************************************************
**/
void DaMiao::Update() {
    // id = (raw_can_data[0]) & 0x0F;
    state = (raw_can_data[0]) >> 4;
    //ERR 表示故障，对应故障类型为：
    //8——超压；
    //9——欠压；
    //A——过电流；
    //B——MOS过温；
    //C——电机线圈过温；
    //D——通讯丢失；
    //E——过载；
    p_int = (raw_can_data[1] << 8) | raw_can_data[2];
    v_int = (raw_can_data[3] << 4) | (raw_can_data[4] >> 4);
    t_int = ((raw_can_data[4] & 0xF) << 8) | raw_can_data[5];
    pos = uint_to_float(p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
    vel = uint_to_float(v_int, V_MIN, V_MAX, 12); // (-30.0,30.0)
    tor = uint_to_float(t_int, T_MIN, T_MAX, 12); // (-10.0,10.0)
    Tmos = (float) (raw_can_data[6]);   //驱动上MOS的平均温度，单位℃
    Tcoil = (float) (raw_can_data[7]);  //电机内部线圈的平均温度，单位℃
}

void DaMiao::EnableMotor() {
    uint8_t data[8];
    CAN_TxHeaderTypeDef canTxHeader;
    uint32_t send_mail;
    canTxHeader.DLC = 0x08;
    canTxHeader.RTR = CAN_RTR_DATA;
    canTxHeader.IDE = CAN_ID_STD;
    canTxHeader.StdId = sendID;
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
    HAL_CAN_AddTxMessage(hcan, &canTxHeader, data, &send_mail);
}

/**
************************************************************************
* @brief:      	disable_motor_mode: 禁用电机模式函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor_id: 电机ID，指定目标电机
* @param[in]:   mode_id:  模式ID，指定要禁用的模式
* @retval:     	void
* @details:    	通过CAN总线向特定电机发送禁用特定模式的命令
************************************************************************
**/
void DaMiao::DisableMotor() {
    uint8_t data[8];
    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;
    HAL_CAN_AddTxMessage(hcan, &canTxHeader, data, (uint32_t*)&send_mail);
}

/**
************************************************************************
* @brief:      	MIT模式下的电机控制函数
* @param[in]:   pos:			位置给定值
* @param[in]:   vel:			速度给定值
* @param[in]:   kp:				位置比例系数
* @param[in]:   kd:				位置微分系数
* @param[in]:   torq:			转矩给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送MIT模式下的控制帧。
************************************************************************
**/
void DaMiao::MIT_Control(float pos, float vel, float kp, float kd, float torq) {
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;

    VAL_LIMIT(torq, -10, 10);

    pos_tmp = float_to_uint(pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(torq, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;

    HAL_CAN_AddTxMessage(hcan, &canTxHeader, data, (uint32_t*)&send_mail);
}

void DaMiao::setSpeedExpect(float ref, float fdk) {
    target_rate = ref;
    Update();
    isLockedRotor = speed_pid.calc(ref, fdk);
    set_tor=speed_pid.output;
}

void DaMiao::setSpeedExpect(float ref) {
    target_rate = ref;
    Update();
    isLockedRotor = speed_pid.calc(target_rate, vel);
    set_tor=speed_pid.output;
}

/**
************************************************************************
* @brief:      	位置速度控制函数
* @param[in]:   vel:			速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送位置速度控制命令
************************************************************************
**/
void DaMiao::PositionSpeedControl(float pos, float vel) {
    uint8_t *pbuf, *vbuf;
    uint8_t data[8];

    pbuf = (uint8_t *) &pos;
    vbuf = (uint8_t *) &vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);

    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);
    HAL_CAN_AddTxMessage(hcan, &canTxHeader, data, (uint32_t*)&send_mail);
}

/**
************************************************************************
* @brief:      	速度控制函数
* @param[in]:   vel: 速度给定值
* @retval:     	void
* @details:    	通过CAN总线向电机发送速度控制命令
************************************************************************
**/
void DaMiao::SpeedControl(float vel) {
    uint8_t *vbuf;
    uint8_t data[4];

    vbuf = (uint8_t *) &vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);

    HAL_CAN_AddTxMessage(hcan, &canTxHeader, data,  (uint32_t*)&send_mail);
}

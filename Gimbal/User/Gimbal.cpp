
#include "Gimbal.hpp"
#include <cstring>


uint8_t flag_shoot=1;
void Gimbal::Update() {
    right_friction_dj_motor->Update();
    left_friction_dj_motor->Update();
    ammunition_dj_motor->Update();
    pitch_dm_motor->Update();
    yaw_dm_motor->Update();
    ControlDJMotor();
    yaw_dm_motor->MIT_Control(3.141593,0,0,0,yaw_dm_motor->set_tor);
    osDelay(1);
    pitch_dm_motor->MIT_Control(3.141593,0,0,0,pitch_dm_motor->set_tor);
}

void Gimbal::SetPitch(float target_pitch, float feedback_pitch_position, float feedback_pitch_speed) {
    if (target_pitch > GIMBAL_MAX_PITCH)target_pitch = GIMBAL_MAX_PITCH;
    if (target_pitch < GIMBAL_MIN_PITCH)target_pitch = GIMBAL_MIN_PITCH;
    this->target_pitch=target_pitch;
    pitch_dm_motor->position_pid.calc(target_pitch, feedback_pitch_position);
    pitch_dm_motor->setSpeedExpect(pitch_dm_motor->position_pid.output, feedback_pitch_speed);
    // pitch_dj_motor->setSpeedExpect(target_pitch, feedback_pitch_speed);
}

void Gimbal::SetYaw(float target_yaw, float feedback_yaw_postion, float feedback_yaw_speed) {
    this->target_yaw=target_yaw;
    yaw_dm_motor->position_pid.calc(target_yaw, feedback_yaw_postion);
    yaw_dm_motor->setSpeedExpect(yaw_dm_motor->position_pid.output, feedback_yaw_speed);
    // yaw_4310.MIT_Control( 0, 0, 0, 0, torq);
}

void Gimbal::Stop(StopType stop_type) {
    switch (stop_type) {
        case StopType::Speed2Zero:
            right_friction_dj_motor->setSpeedExpect(0);
            left_friction_dj_motor->setSpeedExpect(0);
            pitch_dm_motor->setSpeedExpect(0);
            yaw_dm_motor->setSpeedExpect(0);
            ammunition_dj_motor->setSpeedExpect(0);
            board_communicate->Send(BoardCommunicate::Speed2Zero, 0, 0, BoardCommunicate::Follow);
            break;
        case StopType::Torque2Zero:
            right_friction_dj_motor->ZeroTorqueStop();
            left_friction_dj_motor->ZeroTorqueStop();
            pitch_dm_motor->DisableMotor();
            yaw_dm_motor->DisableMotor();
            ammunition_dj_motor->ZeroTorqueStop();
            board_communicate->Send(BoardCommunicate::Torque2Zero, 0, 0, BoardCommunicate::Follow);
            break;
        default:
            break;
    }
}

void Gimbal::Single_Shoot(uint8_t count) {
    if (flag_shoot==1)
    {
        ammunition_target_angle=ammunition_dj_motor->accumlate_angle;
        flag_shoot=0;
    }
    ammunition_target_angle += 91.579 * count;
    if(ammunition_target_angle-ammunition_dj_motor->accumlate_angle>150)
    {
        ammunition_target_angle -= 91.579 * count;
    }
    if(ammunition_dj_motor->isLockedRotor==true){
        ammunition_target_angle = ammunition_dj_motor->accumlate_angle-45.79;
    }
    ammunition_dj_motor->position_pid.calc(ammunition_target_angle,
                                               ammunition_dj_motor->accumlate_angle);
    ammunition_dj_motor->setSpeedExpect(ammunition_dj_motor->position_pid.output);

}

void Gimbal::Continue_Shoot(int speed)
{
    if (ammunition_dj_motor->isLockedRotor ==false)
    {
        ammunition_dj_motor->setSpeedExpect(speed);
    }
    else
    {
        Block();
    }
    flag_shoot=1;
}

void Gimbal::Block()
{

    ammunition_target_angle = ammunition_dj_motor->accumlate_angle-180;
    ammunition_dj_motor->position_pid.calc(ammunition_target_angle,
                                           ammunition_dj_motor->accumlate_angle);
    ammunition_dj_motor->setSpeedExpect(ammunition_dj_motor->position_pid.output);
}
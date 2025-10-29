
#pragma once

#include "DJMotor.hpp"
#include "DaMiao.hpp"
#include "BoardCommunicate.hpp"
#include "ins.h"

#define GIMBAL_MAX_PITCH 48
#define GIMBAL_MIN_PITCH -25

class Gimbal {
private:
    float ammunition_target_angle = 0;
public:
    DJMotor *right_friction_dj_motor, *left_friction_dj_motor, *ammunition_dj_motor;
    DaMiao *yaw_dm_motor, *pitch_dm_motor;
    BoardCommunicate *board_communicate;
    float target_pitch = 0, target_yaw = 0;
    // float bullet_speed = 0;

    enum StopType {
        Speed2Zero,
        Torque2Zero
    };

    Gimbal(DJMotor *right_friction_dj_motor, DJMotor *left_friction_dj_motor,
           DaMiao *pitch_dm_motor,DaMiao *yaw_dm_motor, DJMotor *ammunition_dj_motor,
           BoardCommunicate *board_communicate) : right_friction_dj_motor(right_friction_dj_motor),
                                                  left_friction_dj_motor(left_friction_dj_motor),
                                                  pitch_dm_motor(pitch_dm_motor),
                                                  yaw_dm_motor(yaw_dm_motor),
                                                  ammunition_dj_motor(ammunition_dj_motor),
                                                  board_communicate(board_communicate) {
    }

    void SetPitch(float target_pitch, float feedback_pitch_position, float feedback_pitch_speed);

    void SetYaw(float target_yaw, float feedback_yaw_position, float feedback_yaw_speed);

    void Update();

    void Stop(StopType stop_type);

    void Single_Shoot(uint8_t count);

    void Continue_Shoot(int speed);

    void Block();
};

#pragma once

#include "main.h"
#include "Motor.hpp"
#include "PID.hpp"

void ControlDJMotor();

class DJMotor : public Motor {
public:
    PID speed_pid, position_pid;

    int16_t rotor_raw_angle{};
    int16_t last_rotor_raw_angle{};

    enum Motor_Type {
        M3508 = 1,
        M2006 = 2,
        GM6020 = 3
    };

    uint8_t raw_can_data[8]{};

private:
    Motor_Type motor_type;
    uint8_t which_can;
    uint32_t feedback_id, control_id;
    float reduction_ratio;

    void PopulateControlPackets() const;

public:
    explicit
    DJMotor(uint8_t which_can, uint32_t feedback_id, uint32_t control_id, Motor_Type type,
            float reduction_ratio = 0) : which_can(which_can),
                                         feedback_id(feedback_id), control_id(control_id), motor_type(type) {
        if (reduction_ratio == 0) {
            switch (type) {
                case M3508:
                    this->reduction_ratio = 19;
                case M2006:
                    this->reduction_ratio = 36;
                    break;
                case GM6020:
                    this->reduction_ratio = 1;
            }
        } else {
            this->reduction_ratio = reduction_ratio;
        }
    }

    void Update() override;

    void setSpeedExpect(float ref);

    void setSpeedExpect(float ref, float fdk);

    void ZeroTorqueStop();
};


#ifndef CBOARD_MAHONY_HPP
#define CBOARD_MAHONY_HPP

#include "main.h"

#define RAD2DEG 57.295779513f

class Mahony {
private:
    float twokp, twoki, dt{};
    uint32_t lastTime{};
    float integralFBx{}, integralFBy{}, integralFBz{};

public:
    float q0 = 1, q1{}, q2{}, q3{}, q0_last, q1_last, q2_last, q3_last;

    float yaw{}, pitch{}, roll{};

    float gx, gy, gz;

    Mahony(float _kp = 1, float _ki = 0) : twokp(_kp), twoki(_ki) {
    }

    void Update(float ax, float ay, float az, float gx, float gy, float gz);

    void QuaternionToEuler();
};

#endif //CBOARD_MAHONY_HPP

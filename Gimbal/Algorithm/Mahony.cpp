
#include "Mahony.hpp"
#include "arm_math.h"
#include "bsp_dwt.h"

void Mahony::Update(float ax, float ay, float az, float gx, float gy, float gz) {
    q0_last = q0;
    q1_last = q1;
    q2_last = q2;
    q3_last = q3;
    this->dt = DWT_GetDeltaT(&this->lastTime);
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    arm_sqrt_f32(ax * ax + ay * ay + az * az, &recipNorm);
    if (recipNorm != 0) {
        recipNorm = 1 / recipNorm;
    }
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    halfvx = (q1 * q3 - q0 * q2);
    halfvy = (q0 * q1 + q2 * q3);
    halfvz = (q0 * q0 - 0.5f + q3 * q3);
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);
    if (twoki > 1e-6) {
        integralFBx += twoki * halfex * (dt);
        integralFBy += twoki * halfey * (dt);
        integralFBz += twoki * halfez * (dt);
        gx += integralFBx;
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f;
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }
    gx += twokp * halfex;
    gy += twokp * halfey;
    gz += twokp * halfez;

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    arm_sqrt_f32(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3, &recipNorm);
    if (recipNorm != 0) {
        recipNorm = 1 / recipNorm;
    }
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    float dq[4];
    dq[0] = (q0 - q0_last) / dt;
    dq[1] = (q1 - q1_last) / dt;
    dq[2] = (q2 - q2_last) / dt;
    dq[3] = (q3 - q3_last) / dt;

    this->gx = 2.0 * (-dq[0] * q1 + dq[1] * q0 - dq[2] * q3 + dq[3] * q2);
    this->gy = 2.0 * (-dq[0] * q2 + dq[1] * q3 + dq[2] * q0 - dq[3] * q1);
    this->gz = 2.0 * (-dq[0] * q3 - dq[1] * q2 + dq[2] * q1 + dq[3] * q0);
}

void Mahony::QuaternionToEuler() {
    // 横滚角 (Roll)
    roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD2DEG;
    // 俯仰角 (Pitch)
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabs(sinp) >= 1.0f) {
        pitch = copysignf(M_PI / 2, sinp) * RAD2DEG; // 90度限制
    } else {
        pitch = asinf(sinp) * RAD2DEG;
    }
    // 偏航角 (Yaw)
    yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD2DEG;
}

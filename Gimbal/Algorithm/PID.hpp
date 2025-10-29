#pragma once

#include "main.h"

#define VAL_LIMIT(val, min, max) if(val<=min){val = min;}else if(val>=max){val = max;}

class PID {
private:
    float kp{};
    float ki{};
    float kd{};
    float kf{};
    float componentKpMax{};
    float componentKiMax{};
    float componentKdMax{};
    float outputMax{};

public:
    float ref{};
    float fdb{};
    float last_ref{};
    float err[3]{};
    float componentKp{};
    float componentKi{};
    float componentKd{};
    float output{};
    float error{};
    float diedzone{};
    int stuck_count = 0;
    bool isLocked = false;

    enum PID_Mode {
        ordinary = 0,
        incremental = 1,
        forward=2,
    } mode;

    void reset();

    bool calc();

    bool calc(float _ref, float _fdb);

    void set(PID_Mode modeOperation, float p, float i, float d, float pM, float iM, float dM, float outM);
};

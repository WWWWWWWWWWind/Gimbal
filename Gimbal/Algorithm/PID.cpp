#include <cmath>
#include "PID.hpp"

bool PID::calc() {
    err[2] = err[1];
    err[1] = err[0];
    err[0] = ref - fdb;

    if ((err[0] <= diedzone) && (err[0] > -diedzone)) {
        err[0] = 0;
    }

    if (mode == ordinary) {
        componentKp = kp * err[0];
        VAL_LIMIT (componentKp, -componentKpMax, componentKpMax)

        componentKi += ki * err[0];
        VAL_LIMIT (componentKi, -componentKiMax, componentKiMax)

        componentKd = kd * (err[0] - err[1]) * 0.7f + componentKd * 0.3f;
        VAL_LIMIT (componentKd, -componentKdMax, componentKdMax)
        output = componentKp + componentKi + componentKd;
    }else if (mode == incremental) {
        componentKp = kp * (err[0] - err[1]);
        componentKi = ki * err[0];
        componentKd = kd * (err[0] - 2 * err[1] + err[2]);
        output += componentKp + componentKi + componentKd;
    }else if (mode == forward)
    {
        componentKp = kp * err[0];
        VAL_LIMIT (componentKp, -componentKpMax, componentKpMax)

        componentKi += ki * err[0];
        VAL_LIMIT (componentKi, -componentKiMax, componentKiMax)

        componentKd = kd * (err[0] - err[1]) * 0.7f + componentKd * 0.3f;
        VAL_LIMIT (componentKd, -componentKdMax, componentKdMax)
        output = componentKp + componentKi + componentKd + kf * (ref -last_ref);
    }

    VAL_LIMIT (output, -outputMax, outputMax)

    if (abs(output) < outputMax * 0.3f) {
        isLocked = false;
        stuck_count = 0;
        return isLocked;
    }

    if (abs(err[0]) > abs(fdb* 0.9f)) {
        stuck_count++;
    } else {
        stuck_count = 0;
        isLocked = false;
    }
    if (stuck_count > 100) {
        isLocked = true;
    }
    return isLocked;
}

void PID::reset() {
    ref = 0;
    fdb = 0;

    err[0] = 0;
    err[1] = 0;
    componentKp = 0;
    componentKi = 0;
    componentKd = 0;
    output = 0;
}

void PID::set(PID_Mode modeOperation, float p, float i, float d, float pM, float iM, float dM, float outM) {
    mode = modeOperation;
    kp = p;
    ki = i;
    kd = d;
    componentKpMax = pM;
    componentKiMax = iM;
    componentKdMax = dM;
    outputMax = outM;
}

bool PID::calc(float _ref, float _fdb) {
    ref = _ref;
    fdb = _fdb;
    return this->calc();

}

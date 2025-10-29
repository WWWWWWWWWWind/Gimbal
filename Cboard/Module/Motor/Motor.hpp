#ifndef MOTOR_H
#define MOTOR_H

class Motor {
protected:
    bool connection_lost = true;
    int connection_lost_count = 0;

public:
    float electric_current{};

    float rotor_rate{};
    float rotor_angle{};
    float last_rotor_angle{};
    int rotor_circle_number{};
    float rotor_accumlate_angle{};
    float rotor_target_rate{};

    float rate{};
    float angle{};
    float last_angle{};
    int circle_number{};
    float accumlate_angle{};
    float target_rate{};

    float temperature{};
    bool isLockedRotor = false;

    virtual void Update() {
        connection_lost_count++;
    }

    void connection_active() {
        connection_lost_count = 0;
        connection_lost = false;
    }

    [[nodiscard]] bool isDisconnect() const {
        return connection_lost;
    }
};

#endif //MOTOR_H

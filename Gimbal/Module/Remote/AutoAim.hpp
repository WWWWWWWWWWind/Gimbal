#pragma once

#include <cstring>
#include "main.h"
#include "usbd_cdc_if.h"
#include "bsp_crc.h"

class AutoAim {
public:
    enum Color {
        Red = 0,
        Blue,
    };
    static Color enemy_color;

    struct ReceivePacket {
        uint8_t header = 0xA5;
        uint8_t state: 2; //跟踪状态。0为未再跟随，1为跟随装甲板，2为跟随符。
        uint8_t id: 3;
        uint8_t can_shoot: 1;
        uint8_t reserved: 2;
        float pitch{};
        float yaw{};
        uint16_t checksum = 0;
    } __attribute__((packed)) receive_packet;

    struct SendPacket {
        uint8_t header = 0x5A;
        uint8_t detect_color: 1; // 0-red 1-blue
        uint8_t task_mode: 2;//为瞄准模式。0为自动，1为自瞄，2为打符。默认为1，建议如果后续操作手需要通过键位切换模式时再发其他值
        bool reset_tracker: 1;
        uint8_t is_play: 1;
        uint8_t reserved: 3;
        float roll{};
        float pitch{};
        float yaw{};
        uint16_t  game_time;//比赛开始时间
        float timestamp{};
        float bullet_speed;//弹速
        uint16_t checksum = 0;
    } __attribute__((packed)) send_packet;

    AutoAim() {
        send_packet.bullet_speed = 27.0f;
        send_packet.task_mode = 1;
    }

    void Receive() {
        if (UserRxBufferFS[0] == 0xA5) {
            if (Verify_CRC16_Check_Sum(UserRxBufferFS, sizeof(ReceivePacket))) {
                memcpy(&receive_packet, &UserRxBufferFS, sizeof(ReceivePacket));
            }
        }
    }

    void Send(Color detect_color, float pitch, float yaw) {
        send_packet.header = 0x5A;
        if (detect_color == Blue) {
            send_packet.detect_color = 1;
        } else {
            send_packet.detect_color = 0;
        }
        send_packet.task_mode = 1;
        send_packet.reserved = 0;
        send_packet.pitch = pitch;
        send_packet.yaw = yaw;
        send_packet.checksum = Get_CRC16_Check_Sum((uint8_t *) &send_packet, sizeof(send_packet) - 2, 0xffff);
        CDC_Transmit_FS((uint8_t *) &send_packet, sizeof(SendPacket));
    }
};

extern AutoAim auto_aim;

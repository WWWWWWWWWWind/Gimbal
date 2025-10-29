#pragma once

#include "main.h"

#pragma pack(push)
#pragma pack(2)
class DT7 {
public:
    UART_HandleTypeDef *huart;
    uint8_t RawData[18];


    struct RC {
        int16_t ch0;
        int16_t ch1;
        int16_t ch2;
        int16_t ch3;
        int16_t ch4;
        char s1;
        char s2;
    } rc;

    struct Mouse {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;

    struct Key {
        uint16_t raw_value;
        uint8_t q, w, e, r, a, s, d, f, g, z, x, c,
                v, b, shift, ctrl, mouse_right, mouse_left;
    } key;

    explicit DT7(UART_HandleTypeDef *huart) : huart(huart) {
    }

    void Start() {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, RawData, 18);
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }

    void SBUS2RC() {
        rc.ch0 = (RawData[0] | (RawData[1] << 8)) & 0x07ff;
        rc.ch1 = ((RawData[1] >> 3) | (RawData[2] << 5)) & 0x07ff;
        rc.ch2 = ((RawData[2] >> 6) | (RawData[3] << 2) | (RawData[4] << 10)) & 0x07ff;
        rc.ch3 = ((RawData[4] >> 1) | (RawData[5] << 7)) & 0x07ff;
        rc.ch4 = ((int16_t) RawData[16] | (int16_t) (RawData[17] << 8)) & 0x07FF;
        rc.s1 = ((RawData[5] >> 4) & 0x000C) >> 2;
        rc.s2 = ((RawData[5] >> 4) & 0x0003);
        rc.ch0 -= ((uint16_t) 1024);
        rc.ch1 -= ((uint16_t) 1024);
        rc.ch2 -= ((uint16_t) 1024);
        rc.ch3 -= ((uint16_t) 1024);
        rc.ch4 -= ((uint16_t) 1024);

        mouse.x = RawData[6] | (RawData[7] << 8);
        mouse.y = RawData[8] | (RawData[9] << 8);
        mouse.z = RawData[10] | (RawData[11] << 8);
        mouse.press_l = RawData[12];
        mouse.press_r = RawData[13];
        key.raw_value = RawData[14] | (RawData[15] << 8);
        key.w = key.raw_value & 0x0001;
        key.s = key.raw_value & 0x0002;
        key.a = key.raw_value & 0x0004;
        key.d = key.raw_value & 0x0008;
        key.shift = key.raw_value & 0x0010;
        key.ctrl = key.raw_value & 0x0020;
        key.q = key.raw_value & 0x0040;
        key.e = key.raw_value & 0x0080;
        key.r = key.raw_value & 0x0100;
        key.f = key.raw_value & 0x0200;
        key.g = key.raw_value & 0x0400;
        key.z = key.raw_value & 0x0800;
        key.x = key.raw_value & 0x1000;
        key.c = key.raw_value & 0x2000;
        key.v = key.raw_value & 0x4000;
        key.b = key.raw_value & 0x8000;
    }
};
#pragma pack(pop)
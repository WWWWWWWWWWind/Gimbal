
#include "Referee.hpp"
#include "bsp_crc.h"
#include "DT7.hpp"
#include <cstring>

extern DT7 dt7;

void Referee::Start() {
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rawData, 21);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}


void Referee::Decode(uint8_t *RawData) {
    // 帧头校验
    if (RawData[0] == REMOTE_FRAME_HEADER1 && RawData[1] == REMOTE_FRAME_HEADER2) {
        if (Verify_CRC16_Check_Sum(RawData, 21)) {
            memcpy(&remote_control, RawData, LEN_remote_control);
            mouse.x=remote_control.mouse_x;
            mouse.y=remote_control.mouse_y;
            mouse.z=remote_control.mouse_z;
            mouse.press_l=remote_control.mouse_left;
            mouse.press_r=remote_control.mouse_right ;
            key.w = (remote_control.key & 0x0001)!=0;
            key.s = (remote_control.key & 0x0002)!=0;
            key.a = (remote_control.key & 0x0004)!=0;
            key.d = (remote_control.key & 0x0008)!=0;
            key.shift = (remote_control.key & 0x0010)!=0;
            key.ctrl = (remote_control.key & 0x0020)!=0;
            key.q = (remote_control.key & 0x0040)!=0;
            key.e = (remote_control.key & 0x0080)!=0;
            key.r = (remote_control.key & 0x0100)!=0;
            key.f = (remote_control.key & 0x0200)!=0;
            key.g = (remote_control.key & 0x0400)!=0;
            key.z = (remote_control.key & 0x0800)!=0;
            key.x = (remote_control.key & 0x1000)!=0;
            key.c = (remote_control.key & 0x2000)!=0;
            key.v = (remote_control.key & 0x4000)!=0;
            key.b = (remote_control.key & 0x8000)!=0;
        }
        if (*(RawData + 21) == REMOTE_FRAME_HEADER1 &&*(RawData + 22) == REMOTE_FRAME_HEADER2) {
            Decode(RawData + 21);
        }
    }
}

// void Referee::Key_decode()
// {
//     for (short i = 0; i < 16; i++) {
//         if (remote_control.key & (0x01 << i)) {
//             keyboard[i].pressed = 1;
//         } else {
//             keyboard[i].pressed = 0;
//         }
//     }
//
//     if (remote_control.mouse_left == 0x01) {
//         keyboard[mouse_l].pressed = 1;
//     } else {
//         keyboard[mouse_l].pressed = 0;
//     }
//     if (remote_control.mouse_right == 0x01) {
//         keyboard[mouse_r ].pressed = 1;
//     } else {
//         keyboard[mouse_r].pressed = 0;
//     }
//
//     for (short i = 0; i < 18; i++) {
//         if (keyboard[i].pressed == 1 && keyboard[i].last_state == 0) {
//             keyboard[i].ticked=1;
//         }
//         else
//         {
//             keyboard[i].ticked=0;
//         }
//         keyboard[i].last_state = keyboard[i].pressed;
//     }
// }
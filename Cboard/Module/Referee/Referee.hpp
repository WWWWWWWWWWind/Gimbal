#pragma once

#include "main.h"

#pragma pack(1)

// 新图传
#define REMOTE_FRAME_HEADER1  0xA9
#define REMOTE_FRAME_HEADER2  0x53

#define LEN_HEADER 5  // 帧头长
#define LEN_CMDID 2   // 命令码长度
#define LEN_TAIL 2    // 帧尾CRC16

#define JUDGE_FRAME_HEADER 0xA5

// 命令码ID,用来判断接收的是什么数据
#define ID_power_heat_data 0x0202               // 实时底盘功率和枪口热量数据
#define ID_remote_control 0x0304                // 键鼠遥控数据
#define ID_robot_status 0x0201                  // 机器人性能体系数据

// 命令码数据段长,根据官方协议来定义长度
// 命令码数据段长,根据官方协议来定义长度
typedef enum {
    LEN_game_status = 11, // 0x0001
    LEN_game_result = 1, // 0x0002
    LEN_game_robot_HP = 32, // 0x0003
    LEN_event_data = 4, // 0x0101
    LEN_supply_projectile_action = 4, // 0x0102
    LEN_referee_warning = 3, // 0x0104
    LEN_dart_info = 3, // 0x0105
    LEN_robot_status = 13, // 0x0201
    LEN_power_heat_data = 16, // 0x0202
    LEN_robot_pos = 16, // 0x0203
    LEN_buff = 6, // 0x0204
    LEN_air_support_data = 2, // 0x0205
    LEN_hurt_data = 1, // 0x0206
    LEN_shoot_data = 7, // 0x0207
    LEN_projectile_allowance = 6, // 0x0208
    LEN_rfid_status = 4, // 0x0209
    LEN_dart_client_cmd = 6, // 0x020A
    LEN_ground_robot_position = 40, // 0x020B
    LEN_radar_mark_data = 6, // 0x020C
    LEN_sentry_info = 4, // 0x020D
    LEN_radar_info = 1, // 0x020E
    LEN_robot_interaction_data = 128, // 0x0301
    LEN_custom_robot_data = 30, // 0x0302
    LEN_map_command = 15, // 0x0303
    LEN_remote_control = 21, // 0x0304
    LEN_map_robot_data = 10, // 0x0305
    LEN_custom_client_data = 8, // 0x0306
    LEN_map_data = 103, // 0x0307
    LEN_custom_info = 34, // 0x0308
} JudgeDataLength;

/* ID: 0X0201  Byte: 13    机器人性能体系数据 */
typedef struct {
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;//枪口热量上限
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output: 1;
    uint8_t power_management_chassis_output: 1;
    uint8_t power_management_shooter_output: 1;
} robot_status_t;
/* ID: 0X0202  Byte: 16    实时底盘功率和枪口热量数据 */
typedef struct {
    uint16_t chassis_voltage;
    uint16_t chassis_current;
    float chassis_power;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_1_barrel_heat;
    uint16_t shooter_17mm_2_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;
/* ID: 0x0304  Byte:  12     键鼠遥控数据*/
typedef struct {
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
} remote_control_t;

struct Key {
    uint16_t raw_value{};
    uint8_t q{}, w{}, e{}, r{}, a{}, s{}, d{}, f{}, g{}, z{}, x{}, c{},
            v{}, b{}, shift{}, ctrl{}, mouse_right{}, mouse_left{};
} ;

struct Mouse {
    int16_t x{};
    int16_t y{};
    int16_t z{};
    uint8_t press_l{};
    uint8_t press_r{};
} ;

class Referee {
public:
    UART_HandleTypeDef *huart;
    uint8_t rawData[137]{};
    bool isRenewal = false;

    explicit Referee(UART_HandleTypeDef *huart) : huart(huart) {
    }

    void Decode(uint8_t *RawData);

    void Start();

    void Key_decode();

    robot_status_t robot_status{};
    power_heat_data_t power_heat_data{};
    remote_control_t remote_control{};
    Key key{};
    Mouse mouse{};
};

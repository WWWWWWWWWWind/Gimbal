
#include "bmi088_driver.h"
#include "quaternion_ekf.h"
#include "com_inc.h"
#include "bsp_can.h"
#include "DT7.hpp"
#include "DJMotor.hpp"
#include "DaMiao.hpp"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "ins.h"
#include "spi.h"
#include "BoardCommunicate.hpp"
#include "bsp_usart.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "AutoAim.hpp"
#include "Gimbal.hpp"
#include "Referee.hpp"
#include"lowpass_filter.h"
#include <tgmath.h>
#include "iwdg.h"

extern osThreadId RemoteTaskHandle;
extern osThreadId RefereeTaskHandle;

DaMiao  yaw_4310(&hcan2,0x01,0x10,DaMiao::MIT);//发送ID小于接收ID
DaMiao  pitch_4310(&hcan1,0x02,0x12,DaMiao::MIT);//发送ID小于接收ID
DJMotor ammunition_2006(1, 0x202, 0x200, DJMotor::M2006);//1
DJMotor left_friction_3508(0, 0x206, 0x1FF, DJMotor::M3508, 1);//6
DJMotor right_friction_3508(0, 0x207, 0x1FF, DJMotor::M3508, 1);//7
DT7 dt7(&huart3);
Referee referee(&huart6);
BoardCommunicate board_communicate(&hcan2);
AutoAim auto_aim;
Gimbal gimbal(&right_friction_3508, &left_friction_3508, &pitch_4310, &yaw_4310, &ammunition_2006, &board_communicate);
// float a,b;
uint8_t last_s2, send_vision = 0,moon;
short int r_rising_flag,ctrl_rising_flag, f_rising_flag, mouse_right_rising_flag, c_rising_flag,b_rising_flag, mouse_left_rising_flag,q_rising_flag;
short int pre_key_r,pre_key_ctrl, pre_key_f, pre_key_mouse_right, pre_key_mouse_left, pre_key_c,pre_key_q,pre_key_b; //上一v按键按下状态
short int pre_r_rising_flag,pre_ctrl_rising_flag, pre_f_rising_flag, pre_mouse_right_rising_flag, pre_c_rising_flag,pre_q_rising_flag,pre_b_rising_flag,
          pre_mouse_left_rising_flag;
short int flag_r,flag_ctrl,flag_f, flag_c, flag_q, flag_b,flag_mouse_right, flag_mouse_left,flag_refresh;
short int last_q;
short int vx_set, vy_set, vz_set;
uint16_t shoot_heat, heat_limit,cooling_speed;
int heat_remain;
uint8_t robot_id,remote_disconnected,over_speed,off_speed,grade,life;
float last_yaw,yaw,shoot_frequency,set_shoot_speed=6500,shoot_speed,last_shoot_speed,a,b;
// 初始化滤波器，设置时间常数为0.2
float alpha = 0.2;
AutoAim::Color AutoAim::enemy_color = AutoAim::Blue;

static void RemoteCallBack()
{
    osSignalSet(RemoteTaskHandle, 0x01);
    HAL_UARTEx_ReceiveToIdle_DMA(dt7.huart, dt7.RawData, 18);
    __HAL_DMA_DISABLE_IT(dt7.huart->hdmarx, DMA_IT_HT);
}

static void RefereeCallback()
{
    osSignalSet(RefereeTaskHandle, 0x01);
    HAL_UARTEx_ReceiveToIdle_DMA(referee.huart, referee.rawData, 21);
    __HAL_DMA_DISABLE_IT(referee.huart->hdmarx, DMA_IT_HT);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef CANReceive;
    uint8_t CanRxMsg[8];
    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO0, &CANReceive, CanRxMsg);
    switch (CANReceive.StdId)
    {
    case 0x206:
        for (uint8_t i = 0; i < 8; i++)
        {
            left_friction_3508.raw_can_data[i] = CanRxMsg[i];
            left_friction_3508.connection_active();
        }
        break;
    case 0x207:
        for (uint8_t i = 0; i < 8; i++)
        {
            right_friction_3508.raw_can_data[i] = CanRxMsg[i];
            right_friction_3508.connection_active();
        }
        break;
    case 0x12:
        for (uint8_t i = 0; i < 8; i++)
        {
            pitch_4310.raw_can_data[i] = CanRxMsg[i];
            pitch_4310.connection_active();
        }
        break;

    default:
        break;
    }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef CANReceive;
    uint8_t CanRxMsg[8];
    HAL_CAN_GetRxMessage(hcan, CAN_FILTER_FIFO1, &CANReceive, CanRxMsg);
    switch (CANReceive.StdId)
    {
    case 0x202:
        for (uint8_t i = 0; i < 8; i++)
        {
            ammunition_2006.raw_can_data[i] = CanRxMsg[i];
            ammunition_2006.connection_active();
        }
        break;
    case 0x10:
        for (uint8_t i = 0; i < 8; i++)
        {
            yaw_4310.raw_can_data[i] = CanRxMsg[i];
            yaw_4310.connection_active();
        }
        break;
    case 0x126:
        {
            memcpy(&auto_aim.send_packet.bullet_speed, &CanRxMsg[0], sizeof (auto_aim.send_packet.bullet_speed));
            // memcpy(&bullet_speed , &CanRxMsg[4], sizeof (bullet_speed));
            break;
        }
    case 0x101:
        {
            memcpy(&heat_limit, &CanRxMsg[0], sizeof (heat_limit));
            memcpy(&shoot_heat, &CanRxMsg[2], sizeof (shoot_heat));
            memcpy(&robot_id, &CanRxMsg[4], sizeof (robot_id));
            memcpy(&grade,&CanRxMsg[5], sizeof (grade));
            memcpy(&cooling_speed,&CanRxMsg[6], sizeof (cooling_speed));
            if (robot_id<11)
            {AutoAim::enemy_color = AutoAim::Blue; }
            else AutoAim::enemy_color = AutoAim::Red;
            break;
        }
    case 0x102:
        {
            memcpy(&shoot_speed, &CanRxMsg[0], sizeof (shoot_speed));
            memcpy(&life, &CanRxMsg[4], sizeof (life));
        }
    default:
        {
            break;
        }
    }
}
void BSP()
{
    USART_CallBack[2] = RemoteCallBack; // 0对应串口1,1对应串口2,以此类推
    USART_CallBack[5] = RefereeCallback;
    DWT_Init(168);
    while (BMI088_init(&hspi1, 1) != BMI088_NO_ERROR);
    BspCanInit();
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//蜂鸣器
    yaw_4310.speed_pid.set(PID::ordinary, 3.1, 0.0, 0, 10, 5, 3, 10);
    yaw_4310.position_pid.set(PID::ordinary, 0.3, 0.0005, 0, 10, 5, 3, 10);
    pitch_4310.speed_pid.set(PID::ordinary, 2.5, 0.02, 0, 10, 5, 3, 10);
    pitch_4310.position_pid.set(PID::ordinary, 0.8, 0.0, 0, 10, 5, 3, 20);
    // ammunition_2006.speed_pid.set(PID::ordinary, 240, 0, 1.5, 9000, 500, 1000, 10000);
    // ammunition_2006.position_pid.set(PID::ordinary, 5, 0, 0, 300, 0, 0, 300);
    ammunition_2006.speed_pid.set(PID::ordinary, 50, 0.01, 1.5, 9000, 500, 1000, 9500);
    ammunition_2006.position_pid.set(PID::ordinary, 3, 0, 0, 300, 0, 0, 300);
    left_friction_3508.speed_pid.set(PID::ordinary, 15, 0, 0, 15000, 3500, 1500, 16000);
    right_friction_3508.speed_pid.set(PID::ordinary, 15, 0, 0, 15000, 3500, 1500, 16000);
}

void StartDefaultTask(void const* argument)
{
    MX_USB_DEVICE_Init();
    for (;;)
    {
        HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
        osDelay(500);
    }
}

void StartIMU(void const* argument)
{
    INS_Init(); //改过 bmi088_driver.c 对应坐标系
    for (;;)
    {
        INS_Task();
        osDelay(1);
    }
}

void StartRemote(void const* argument)
{
    dt7.Start();
    for (;;)
    {
        // osSignalWait(0x01, 0xFFFFFFFF);
        // 设置超时等待
        osEvent event = osSignalWait(0x01, 50);
        if (event.status == osEventTimeout)
        {
            // 遥控器断联处理
            remote_disconnected=1;
        }
        else
        {
            remote_disconnected=0;
            dt7.SBUS2RC();
        }
        osDelay(1);
    }
}

void StartGimbal(void const* argument)
{
    initializeFilter(&Filter, alpha);
    for (;;)
    {
        if (remote_disconnected==1)
        {
            Key_mode();
        } else
        {
            RC_mode();
        }
        gimbal.Update();
        osDelay(1);
    }
}

void StartVision(void const* argument)
{
    MX_USB_DEVICE_Init();
    for (;;)
    {
        osSignalWait(0x01, 0xFFFFFFFF); //改过 usbd_cdc_if.c 的25、26、267行
        auto_aim.Receive();
        osDelay(1);
    }
}

void SendVision(void const* argument)
{
    for (;;)
    {
        auto_aim.Send(AutoAim::enemy_color, INS.Pitch * 0.01745329252f, INS.YawTotalAngle * 0.01745329252f);
        osDelay(5);//8
    }
}

void StartBoardCommunicate(void const* argument)
{
    for (;;)
    {
        if (board_communicate.chassis_type == BoardCommunicate::TorqueZero_type)
        {
            board_communicate.Send(BoardCommunicate::Torque2Zero, 0, 0, BoardCommunicate::Follow);
        }
        else if (board_communicate.chassis_type == BoardCommunicate::SpeedZero_type)
        {
            board_communicate.Send(BoardCommunicate::Speed2Zero, 0, 0, BoardCommunicate::Follow);
        }
        else if (board_communicate.chassis_type == BoardCommunicate::Follow_type)
        {
            board_communicate.Send(BoardCommunicate::Normal, vx_set, vy_set, BoardCommunicate::Follow);
        }
        else if (board_communicate.chassis_type == BoardCommunicate::F_Gyro_type)
        {
            board_communicate.Send(BoardCommunicate::Normal, vx_set, vy_set, BoardCommunicate::Counterclockwise);
        }
        else if (board_communicate.chassis_type == BoardCommunicate::Gyro_type)
        {
            board_communicate.Send(BoardCommunicate::Normal, vx_set, vy_set, BoardCommunicate::Clockwise);
        }
        else
        {
            board_communicate.Send(BoardCommunicate::Torque2Zero, 0, 0, BoardCommunicate::Follow);
        }
        memcpy(&board_communicate.RawData[1][0], &yaw_4310.pos, sizeof(float));
        board_communicate.Send(1);
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(1);
    }
}

void StartReferee(void const* argument)
{
    referee.Start();
    for (;;)
    {
        osSignalWait(0x01, 0xFFFFFFFF);
        referee.Decode(referee.rawData);
        osDelay(1);
    }
}

void RC_mode()
{
    if (dt7.rc.s1 == 1||dt7.rc.s1 == 3)
    {
        //使能电机
        yaw_4310.EnableMotor();
        pitch_4310.EnableMotor();
        osDelay(1);
        //设置底盘运动模式
        if (dt7.rc.ch4 == 660)
        {
            board_communicate.chassis_type = BoardCommunicate::F_Gyro_type;
        }
        else if (dt7.rc.ch4 == -660)
        {
            board_communicate.chassis_type = BoardCommunicate::Gyro_type;
        }
        else
        {
            board_communicate.chassis_type = BoardCommunicate::Follow_type;
        }
        vx_set = dt7.rc.ch3 ;
        vy_set = -dt7.rc.ch2 ;
        //摩擦轮
        if (dt7.rc.s2 != 2)
        {
            // right_friction_3508.setSpeedExpect(1000);//6000
            // left_friction_3508.setSpeedExpect(-1000);
            //判断是否需要降速
            if (shoot_speed>=24.5)
            {
                over_speed=1;
            }
            else
            {
                over_speed=0;
            }
            //判断是否需要增速
            if (shoot_speed>18 && shoot_speed<=21.3)
            {
                off_speed=1;
            }
            else
            {
                off_speed=0;
            }
            //判断值是否更新
            if (shoot_speed==last_shoot_speed)
            {
                flag_refresh=0;
            }
            else
            {
                flag_refresh=1;
            }

            if (over_speed==1 && flag_refresh==1)
            {
                set_shoot_speed -=200;
            }
            else if (off_speed==1 && flag_refresh==1)
            {
                set_shoot_speed +=100;
            }
            //防止反转
            if (set_shoot_speed<0)
            {
                set_shoot_speed=0;
            }
            right_friction_3508.setSpeedExpect(set_shoot_speed);//6000
            left_friction_3508.setSpeedExpect(-set_shoot_speed);
            last_shoot_speed=shoot_speed;
        }
        else
        {
            right_friction_3508.setSpeedExpect(0);
            left_friction_3508.setSpeedExpect(0);
        }
        //单发
        if (dt7.rc.s1==1)
        {
            if (dt7.rc.s2 != 2)
            {
                if (last_s2 == 3 && dt7.rc.s2 == 1)
                {
                    gimbal.Single_Shoot(1);
                }
                else
                {
                    gimbal.Single_Shoot(0);
                }
                last_s2 = dt7.rc.s2;
            }
            else
            {
                ammunition_2006.setSpeedExpect(0);
            }
        }
        //连发
        else if (dt7.rc.s1==3)
        {
            if (dt7.rc.s2 == 1 )
            // if (dt7.rc.s2 == 1 && auto_aim.receive_packet.can_shoot==1)
            {
                Heat_control();
                gimbal.Continue_Shoot(shoot_frequency*15.26);//x*60/(19/58*12)
            }
            else
            {
                ammunition_2006.setSpeedExpect(0);
            }
        }
        //自瞄
        if (auto_aim.receive_packet.state==1)
        {
            // a=auto_aim.receive_packet.pitch * 57.2957795f;
            // b=auto_aim.receive_packet.yaw * 57.2957795f;
            // gimbal.target_pitch = filterValue(&Filter, auto_aim.receive_packet.pitch * 57.2957795f);
            // yaw = filterValue(&Filter, auto_aim.receive_packet.yaw * 57.2957795f);
            gimbal.target_pitch = -auto_aim.receive_packet.pitch * 57.2957795f;
            yaw = auto_aim.receive_packet.yaw * 57.2957795f;
            if (yaw-last_yaw>180)
            {
                gimbal.target_yaw=yaw-last_yaw-360+INS.YawTotalAngle;
            }
            else if (yaw-last_yaw<-180)
            {
                gimbal.target_yaw=yaw-last_yaw+360+INS.YawTotalAngle;
            }
            else
            {
                gimbal.target_yaw=yaw-last_yaw+INS.YawTotalAngle;
            }
            last_yaw=INS.Yaw;
            gimbal.SetPitch(gimbal.target_pitch, -INS.Pitch, -INS.Gyro[0] );//弧度制
            gimbal.SetYaw(gimbal.target_yaw, INS.YawTotalAngle, INS.Gyro[2] );//弧度制
            auto_aim.receive_packet.state=0;
        }
        else
        {

            gimbal.target_yaw -= dt7.rc.ch0 / 660.0f;
            gimbal.target_pitch += dt7.rc.ch1 / 3300.0f;
            gimbal.SetPitch(gimbal.target_pitch, -INS.Pitch, -INS.Gyro[0] );//弧度制
            gimbal.SetYaw(gimbal.target_yaw, INS.YawTotalAngle, INS.Gyro[2] );//弧度制
        }
    }
    else if (dt7.rc.s1 == 2)
    {
        gimbal.Stop(Gimbal::Torque2Zero);
        board_communicate.chassis_type = BoardCommunicate::TorqueZero_type;
    }
}
void Key_mode()
{
    yaw_4310.EnableMotor();
    pitch_4310.EnableMotor();
    osDelay(1);

    if (life == 0)
    {
        board_communicate.chassis_type = BoardCommunicate::Follow_type;
        // flag_c = 0;//上电还是原来的状态，再次按c一定是跟随状态
        flag_f = 0;
        flag_r = 0;//上电默认不开火控
        gimbal.target_pitch=-INS.Pitch;
        gimbal.target_yaw=INS.YawTotalAngle;
        yaw_4310.set_tor=0;
        pitch_4310.set_tor=0;
    }
    else
    {
        if (referee.key.w == 1)
    {
        vx_set=300+20*grade;
    }
    else if (referee.key.s == 1)
    {
        vx_set=-300-20*grade;
    }
    else
    {
        vx_set = 0;
    }

    if (referee.key.a == 1)
    {
        vy_set=300+20*grade;
    }
    else if (referee.key.d == 1)
    {
        vy_set=-300-20*grade;
    }
    else
    {
        vy_set = 0;
    }
    //加速
    // if (referee.key.shift == 1)
    // {
    //     vx_set=vx_set*1.2;
    //     vy_set=vy_set*1.2;
    // }

    //小陀螺
    c_rising_flag = referee.key.c - pre_key_c;
    if (c_rising_flag == 1 && flag_c == 0)
    {
        board_communicate.chassis_type = BoardCommunicate::Follow_type;
        flag_c = 1;
    }
    else if (c_rising_flag == 1 && flag_c == 1)
    {
        board_communicate.chassis_type = BoardCommunicate::Gyro_type;
        flag_c = 0;
    }
    pre_key_c = referee.key.c;

    //开启摩擦轮
    f_rising_flag = referee.key.f - pre_key_f;
    if (f_rising_flag == 1 && flag_f == 0)
    {
        flag_f = 1;
    }
    else if (f_rising_flag == 1 && flag_f == 1)
    {
        flag_f = 0;
    }
    pre_key_f = referee.key.f;
    if (flag_f)
    {
        //判断是否需要降速
        if (shoot_speed>=24.5)
        {
            over_speed=1;
        }
        else
        {
            over_speed=0;
        }
        //判断值是否更新
        if (shoot_speed==last_shoot_speed)
        {
            flag_refresh=0;
        }
        else
        {
            flag_refresh=1;
        }

        if (over_speed==1 && flag_refresh==1)
        {
            set_shoot_speed -=200;
        }
        //防止反转
        if (set_shoot_speed<0)
        {
            set_shoot_speed=0;
        }
        right_friction_3508.setSpeedExpect(set_shoot_speed);//6000
        left_friction_3508.setSpeedExpect(-set_shoot_speed);
        last_shoot_speed=shoot_speed;
    }else
    {
        gimbal.right_friction_dj_motor->setSpeedExpect(0);
        gimbal.left_friction_dj_motor->setSpeedExpect(0);
    }

    //火控
    r_rising_flag = referee.key.r - pre_key_r;
    if (r_rising_flag == 1 && flag_r == 0)
    {
        flag_r = 1;
    }
    else if (r_rising_flag == 1 && flag_r == 1)
    {
        flag_r = 0;
    }
    pre_key_r = referee.key.r;

    //连发
    if ( flag_r == 1)
    {
        if (referee.mouse.press_l == 1 && auto_aim.receive_packet.can_shoot==1)
        {
            Heat_control();
            gimbal.Continue_Shoot(shoot_frequency*15.26);
        }
        else
        {
            ammunition_2006.setSpeedExpect(0);
        }

    }
    else if (flag_r == 0)
    {
        if (referee.mouse.press_l == 1)
        {
            Heat_control();
            gimbal.Continue_Shoot(shoot_frequency*15.26);
        }
        else
        {
            ammunition_2006.setSpeedExpect(0);
        }
    }


    //单发
    b_rising_flag = referee.key.b - pre_key_b;
    if (referee.mouse.press_l == 0)
    {
        if (b_rising_flag == 1 )
        {
            gimbal.Single_Shoot(1);
        }
        else
        {
            gimbal.Single_Shoot(0);
        }
    }
    pre_key_b = referee.key.b;

    //自瞄
    if (referee.mouse.press_r == 1)
    {
        if (auto_aim.receive_packet.state == 1)
        {
            gimbal.target_pitch = -filterValue(&Filter, auto_aim.receive_packet.pitch * 57.2957795f);
            yaw = auto_aim.receive_packet.yaw * 57.2957795f;

            if (yaw-last_yaw>180)
            {
                gimbal.target_yaw=yaw-last_yaw-360+INS.YawTotalAngle;
            }
            else if (yaw-last_yaw<-180)
            {
                gimbal.target_yaw=yaw-last_yaw+360+INS.YawTotalAngle;
            }
            else
            {
                gimbal.target_yaw=yaw-last_yaw+INS.YawTotalAngle;
            }
            last_yaw=INS.Yaw;
            gimbal.SetPitch(gimbal.target_pitch, -INS.Pitch, -INS.Gyro[0] );
            gimbal.SetYaw(gimbal.target_yaw, INS.YawTotalAngle, INS.Gyro[2] );
            auto_aim.receive_packet.state=0;
        }
        else
        {
            gimbal.target_yaw -= referee.mouse.x / 200.0f;
            gimbal.target_pitch += referee.mouse.y / 700.0f;
            gimbal.SetPitch(gimbal.target_pitch, -INS.Pitch, -INS.Gyro[0] );
            gimbal.SetYaw(gimbal.target_yaw, INS.YawTotalAngle, INS.Gyro[2] );
        }

    }else
    {
        auto_aim.receive_packet.state=0;
        gimbal.target_yaw -= referee.mouse.x / 200.0f;
        gimbal.target_pitch += referee.mouse.y / 700.0f;
        gimbal.SetPitch(gimbal.target_pitch, -INS.Pitch, -INS.Gyro[0] );
        gimbal.SetYaw(gimbal.target_yaw, INS.YawTotalAngle, INS.Gyro[2] );
    }
    }
    //UI 1,1,0,1,1
    //g刷新UI，f摩擦轮是否开启，c小陀螺是否开启，shift开启超电
    board_communicate.Send_UI(referee.key.g,flag_f,flag_c,referee.key.shift,flag_r);
}

void Servo_control()
{
        if (dt7.rc.s2==3)
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,1500);
        else if (dt7.rc.s2==1)
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,500);
        else if (dt7.rc.s2==2)
            __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,2500);
}

void Heat_control()
{
    heat_remain=heat_limit -shoot_heat;
    if (heat_remain>=100)
    {
        shoot_frequency=16;
    }
    else if (40<heat_remain && heat_remain<100)
    {
        shoot_frequency=(16*(40-heat_remain)+cooling_speed/10*(heat_remain-100))/(40-100);
    }
    else if (20<heat_remain && heat_remain<=40)
    {
        shoot_frequency=cooling_speed/10;
    }
    else if (heat_remain<=20)
    {
        shoot_frequency=0;
    }
    VAL_LIMIT(shoot_frequency,0,18);
}
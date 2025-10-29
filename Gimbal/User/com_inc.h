
#ifndef ROTATE_MAIN_H
#define ROTATE_MAIN_H
#ifdef __cplusplus
extern "C" {
#endif

void BSP();

void StartDefaultTask(void const *argument);

void StartIMU(void const *argument);

void StartRemote(void const *argument);

void StartGimbal(void const *argument);

void StartVision(void const *argument);

void StartBoardCommunicate(void const *argument);

void SendVision(void const *argument);

void StartReferee(void const *argument);

void RC_mode();

void Key_mode();

void Servo_control();

void Heat_control();

#ifdef __cplusplus
}
#endif
#endif //ROTATE_MAIN_H

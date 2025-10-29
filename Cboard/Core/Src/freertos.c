/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "com_inc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId IMUTaskHandle;
osThreadId RemoteTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId VisionTaskHandle;
osThreadId BoardCommunicatHandle;
osThreadId ShootTaskHandle;
osThreadId RefereeTaskHandle;
osThreadId SendTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartIMU(void const * argument);
void StartRemote(void const * argument);
void StartGimbal(void const * argument);
void StartVision(void const * argument);
void StartBoardCommunicate(void const * argument);
void StartShoot(void const * argument);
void StartReferee(void const * argument);
void SendVision(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IMUTask */
  osThreadDef(IMUTask, StartIMU, osPriorityHigh, 0, 512);
  IMUTaskHandle = osThreadCreate(osThread(IMUTask), NULL);

  /* definition and creation of RemoteTask */
  osThreadDef(RemoteTask, StartRemote, osPriorityRealtime, 0, 256);
  RemoteTaskHandle = osThreadCreate(osThread(RemoteTask), NULL);

  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, StartGimbal, osPriorityHigh, 0, 512);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of VisionTask */
  osThreadDef(VisionTask, StartVision, osPriorityHigh, 0, 512);
  VisionTaskHandle = osThreadCreate(osThread(VisionTask), NULL);

  /* definition and creation of BoardCommunicat */
  osThreadDef(BoardCommunicat, StartBoardCommunicate, osPriorityRealtime, 0, 256);
  BoardCommunicatHandle = osThreadCreate(osThread(BoardCommunicat), NULL);

  /* definition and creation of ShootTask */
  osThreadDef(ShootTask, StartShoot, osPriorityIdle, 0, 256);
  ShootTaskHandle = osThreadCreate(osThread(ShootTask), NULL);

  /* definition and creation of RefereeTask */
  osThreadDef(RefereeTask, StartReferee, osPriorityHigh, 0, 256);
  RefereeTaskHandle = osThreadCreate(osThread(RefereeTask), NULL);

  /* definition and creation of SendTask */
  osThreadDef(SendTask, SendVision, osPriorityHigh, 0, 128);
  SendTaskHandle = osThreadCreate(osThread(SendTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartIMU */
/**
* @brief Function implementing the IMUTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMU */
__weak void StartIMU(void const * argument)
{
  /* USER CODE BEGIN StartIMU */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartIMU */
}

/* USER CODE BEGIN Header_StartRemote */
/**
* @brief Function implementing the RemoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemote */
__weak void StartRemote(void const * argument)
{
  /* USER CODE BEGIN StartRemote */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartRemote */
}

/* USER CODE BEGIN Header_StartGimbal */
/**
* @brief Function implementing the GimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbal */
__weak void StartGimbal(void const * argument)
{
  /* USER CODE BEGIN StartGimbal */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartGimbal */
}

/* USER CODE BEGIN Header_StartVision */
/**
* @brief Function implementing the VisionTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartVision */
__weak void StartVision(void const * argument)
{
  /* USER CODE BEGIN StartVision */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartVision */
}

/* USER CODE BEGIN Header_StartBoardCommunicate */
/**
* @brief Function implementing the BoardCommunicat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBoardCommunicate */
__weak void StartBoardCommunicate(void const * argument)
{
  /* USER CODE BEGIN StartBoardCommunicate */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartBoardCommunicate */
}

/* USER CODE BEGIN Header_StartShoot */
/**
* @brief Function implementing the ShootTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartShoot */
__weak void StartShoot(void const * argument)
{
  /* USER CODE BEGIN StartShoot */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartShoot */
}

/* USER CODE BEGIN Header_StartReferee */
/**
* @brief Function implementing the RefereeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReferee */
__weak void StartReferee(void const * argument)
{
  /* USER CODE BEGIN StartReferee */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartReferee */
}

/* USER CODE BEGIN Header_SendVision */
/**
* @brief Function implementing the SendTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SendVision */
__weak void SendVision(void const * argument)
{
  /* USER CODE BEGIN SendVision */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SendVision */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

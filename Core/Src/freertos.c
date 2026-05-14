/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dji_motor.h"
#include "daemon.h"
#include "robot.h"
#include "chassis.h"
#include "robot_cmd.h"
#include "gimbal.h"
#include "shoot.h"
#include "ins_task.h"
#include "referee_task.h"
#include "buzzer.h"
#include "power_control.h"
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
osThreadId MOTORTASKHandle;
osThreadId DAEMONTASKHandle;
osThreadId ROBOTTASKHandle;
osThreadId INS_TASKHandle;
osThreadId StartUI_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartMOTORTASK(void const * argument);
void StartDAEMONTASK(void const * argument);
void StartROBOTTASK(void const * argument);
void ins_task(void const * argument);
void StartUITASK(void const * argument);

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

  /* definition and creation of MOTORTASK */
  osThreadDef(MOTORTASK, StartMOTORTASK, osPriorityNormal, 0, 256);
  MOTORTASKHandle = osThreadCreate(osThread(MOTORTASK), NULL);

  /* definition and creation of DAEMONTASK */
  osThreadDef(DAEMONTASK, StartDAEMONTASK, osPriorityNormal, 0, 521);
  DAEMONTASKHandle = osThreadCreate(osThread(DAEMONTASK), NULL);

  /* definition and creation of ROBOTTASK */
  osThreadDef(ROBOTTASK, StartROBOTTASK, osPriorityNormal, 0, 1024);
  ROBOTTASKHandle = osThreadCreate(osThread(ROBOTTASK), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, ins_task, osPriorityNormal, 0, 1024);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of StartUI_TASK */
  osThreadDef(StartUI_TASK, StartUITASK, osPriorityNormal, 0, 512);
  StartUI_TASKHandle = osThreadCreate(osThread(StartUI_TASK), NULL);

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
void StartDefaultTask(void const * argument)
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

/* USER CODE BEGIN Header_StartMOTORTASK */
/**
* @brief Function implementing the MOTORTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMOTORTASK */
void StartMOTORTASK(void const * argument)
{
  /* USER CODE BEGIN StartMOTORTASK */
  /* Infinite loop */
  while(1)
  {	
	  DJIMotorControl();
    PowerControl();
	  osDelay(1); 
  }
	  
  /* USER CODE END StartMOTORTASK */
}

/* USER CODE BEGIN Header_StartDAEMONTASK */
/**
* @brief Function implementing the DAEMONTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDAEMONTASK */
void StartDAEMONTASK(void const * argument)
{
  /* USER CODE BEGIN StartDAEMONTASK */
  /* Infinite loop */
 while (1)
 {
  DaemonTask();
  osDelay(10);
 }
 
  /* USER CODE END StartDAEMONTASK */
}

/* USER CODE BEGIN Header_StartROBOTTASK */
/**
* @brief Function implementing the ROBOTTASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartROBOTTASK */
void StartROBOTTASK(void const * argument)
{
  /* USER CODE BEGIN StartROBOTTASK */
  /* Infinite loop */
while (1)
{
//    BuzzerTask();
    RobotCMDTask();
    ChassisTask();
    GimbalTask();
    ShootTask();
  	osDelay(5);
}

  /* USER CODE END StartROBOTTASK */
}

/* USER CODE BEGIN Header_ins_task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ins_task */
void ins_task(void const * argument)
{
  /* USER CODE BEGIN ins_task */
  /* Infinite loop */
  while (1)
  {
    // 1kHz
    INS_Task();
 //  VisionSend(); // ������ɺ����Ӿ�����
    osDelay(1);
  }
  /* USER CODE END ins_task */
}

/* USER CODE BEGIN Header_StartUITASK */
/**
* @brief Function implementing the StartUI_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUITASK */
void StartUITASK(void const * argument)
{
  /* USER CODE BEGIN StartUITASK */
  /* Infinite loop */
  while (1)
  {
    My_UI_init();
    Referee_Interactive_task();        
  }
  /* USER CODE END StartUITASK */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

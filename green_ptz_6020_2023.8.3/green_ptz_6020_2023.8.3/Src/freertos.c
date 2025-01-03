/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "led_flow_task.h"
#include "bsp_buzzer.h"
#include "zn_chassis_task.h"
#include "zn_gimbal_task.h"
#include "zn_shoot_task.h"
#include "referee_usart_task.h"
#include "voltage_task.h"
#include "INS_task.h"
#include "zn_display_task.h"
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
osThreadId AppTaskCreateHandle;

osThreadId led_RGB_flow_handle;
osThreadId chassisTaskHandle;
osThreadId referee_usart_task_handle;
osThreadId battery_voltage_handle;
osThreadId imuTaskHandle;
osThreadId gimbalTaskHandle;
osThreadId shootTaskHandle;
osThreadId c;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void AppTaskCreate_task(void const * argument);


extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

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

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN RTOS_THREADS */
  /*������ʼ�����ɸ����񴴽�������*/
  osThreadDef(AppTaskCreate, AppTaskCreate_task, osPriorityNormal, 0, 128);
  AppTaskCreateHandle = osThreadCreate(osThread(AppTaskCreate), NULL);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_test_task */
/**
  * @brief  Function implementing the test thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_test_task */
void AppTaskCreate_task(void const * argument)
{
  /*���� LED��˸ ����*/
  osThreadDef(led, led_RGB_flow_task, osPriorityNormal, 0, 256);//0
  led_RGB_flow_handle = osThreadCreate(osThread(led), NULL);
  
  /*���� ���� ����*/
  osThreadDef(ZN_chassis_task, ZN_chassis_task, osPriorityAboveNormal, 0, 1024);//+2
  chassisTaskHandle = osThreadCreate(osThread(ZN_chassis_task), NULL);

  /*���� ��̨ ����*/
    osThreadDef(gimbalTask, ZN_gimbal_task, osPriorityHigh, 0, 512);//+2
    gimbalTaskHandle = osThreadCreate(osThread(gimbalTask), NULL);
    
  /*���� ��� ����*/
    osThreadDef(shootTask, ZN_shoot_task, osPriorityHigh, 0, 512);//+2
    shootTaskHandle = osThreadCreate(osThread(shootTask), NULL);    
  
  /*���� ����ϵͳ ����*/
  osThreadDef(REFEREE, referee_usart_task, osPriorityNormal, 0, 128);//0
  referee_usart_task_handle = osThreadCreate(osThread(REFEREE), NULL);
  
  /*���� ��ѹ��� ����*/
  osThreadDef(BATTERY_VOLTAGE, battery_voltage_task, osPriorityNormal, 0, 128);//0
  battery_voltage_handle = osThreadCreate(osThread(BATTERY_VOLTAGE), NULL); 

  /*���� �ǶȽ��� ����*/
  osThreadDef(imuTask, INS_task, osPriorityRealtime, 0, 1024);//+3
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
  
  /*���� ��ʾ ����*/
   osThreadDef(Display, ZN_display_task, osPriorityLow, 0, 256);//-2
   osThreadId oled_handle = osThreadCreate(osThread(Display), NULL);  
  
  vTaskDelete(NULL);//ɾ������
  
  while(1);//����ִ�е�����
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

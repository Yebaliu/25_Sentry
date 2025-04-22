/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
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
#include "usbd_cdc_if.h"
#include "CAN_receive&send.h"
#include "DBUS_remote_control.h"
#include "LED_control.h"
#include "IMU_updata.h"
#include "PWM_control.h"
#include "Stm32_time.h"
#include "string.h"

#include "chassis_move.h" //普通底盘
#include "gimbal.h"
#include "math.h"

#include "USB_VirCom.h"

#include "Global_status.h"
#include "Error_detect.h"

#include "cap_ctl.h"
#include "referee_handle_pack.h"
#include "referee_usart_task.h"
#include "fifo.h"
#include "AHRS_MiddleWare.h"
#include "CAN_receive&send.h"
#include "Laser.h"

#include "vofa.h"

#include "wwdg.h"
#include "stm32f4xx_hal_wwdg.h"

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

extern struct IMU_t IMU_data;
extern uint8_t coverop;
extern pid_t motor_speed[4];
extern float Time_delay_trs;

/* USER CODE END Variables */
/* Definitions for Flash_LED_Task */
osThreadId_t Flash_LED_TaskHandle;
const osThreadAttr_t Flash_LED_Task_attributes = {
    .name = "Flash_LED_Task",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for CAN_sendTask */
osThreadId_t CAN_sendTaskHandle;
const osThreadAttr_t CAN_sendTask_attributes = {
    .name = "CAN_sendTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityRealtime7,
};
/* Definitions for RemoteTask */
osThreadId_t RemoteTaskHandle;
const osThreadAttr_t RemoteTask_attributes = {
    .name = "RemoteTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for ChassisTask */
osThreadId_t ChassisTaskHandle;
const osThreadAttr_t ChassisTask_attributes = {
    .name = "ChassisTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for GimbalTask */
osThreadId_t GimbalTaskHandle;
const osThreadAttr_t GimbalTask_attributes = {
    .name = "GimbalTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for ErrorDetectTask */
osThreadId_t ErrorDetectTaskHandle;
const osThreadAttr_t ErrorDetectTask_attributes = {
    .name = "ErrorDetectTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for RefereeTask */
osThreadId_t RefereeTaskHandle;
const osThreadAttr_t RefereeTask_attributes = {
    .name = "RefereeTask",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for UI_Refresh */
osThreadId_t UI_RefreshHandle;
const osThreadAttr_t UI_Refresh_attributes = {
    .name = "UI_Refresh",
    .stack_size = 256 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Flash_LED_Task_callback(void *argument);
void CAN_sendTask_callback(void *argument);
void RemoteTask_callback(void *argument);
void ChassisTask_callback(void *argument);
void GimbalTask_callback(void *argument);
void ErrorDetectTask_callback(void *argument);
void RefereeTask_callback(void *argument);
void UI_Refresh_callback(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook(void)
{
  /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
  to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
  task. It is essential that code added to this hook function never attempts
  to block in any way (for example, call xQueueReceive() with a block time
  specified, or call vTaskDelay()). If the application makes use of the
  vTaskDelete() API function (as this demo application does) then it is also
  important that vApplicationIdleHook() is permitted to return to its calling
  function, because it is the responsibility of the idle task to clean up
  memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
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
  /* creation of Flash_LED_Task */
  Flash_LED_TaskHandle = osThreadNew(Flash_LED_Task_callback, NULL, &Flash_LED_Task_attributes);

  /* creation of CAN_sendTask */
  CAN_sendTaskHandle = osThreadNew(CAN_sendTask_callback, NULL, &CAN_sendTask_attributes);

  /* creation of RemoteTask */
  RemoteTaskHandle = osThreadNew(RemoteTask_callback, NULL, &RemoteTask_attributes);

  /* creation of ChassisTask */
  ChassisTaskHandle = osThreadNew(ChassisTask_callback, NULL, &ChassisTask_attributes);

  /* creation of GimbalTask */
  GimbalTaskHandle = osThreadNew(GimbalTask_callback, NULL, &GimbalTask_attributes);

  /* creation of ErrorDetectTask */
  ErrorDetectTaskHandle = osThreadNew(ErrorDetectTask_callback, NULL, &ErrorDetectTask_attributes);

  /* creation of RefereeTask */
  RefereeTaskHandle = osThreadNew(RefereeTask_callback, NULL, &RefereeTask_attributes);

  /* creation of UI_Refresh */
  UI_RefreshHandle = osThreadNew(UI_Refresh_callback, NULL, &UI_Refresh_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_Flash_LED_Task_callback */
/**
 * @brief  Function implementing the Flash_LED_Task thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Flash_LED_Task_callback */
void Flash_LED_Task_callback(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN Flash_LED_Task_callback */
  /* Infinite loop */
  for (;;)
  {
    led_show(GREEN);
    osDelay(500);
    led_show(RED);
    osDelay(500);
    led_show(BLUE);
    osDelay(500);
  }
  /* USER CODE END Flash_LED_Task_callback */
}

/* USER CODE BEGIN Header_CAN_sendTask_callback */
/**
 * @brief Function implementing the CAN_sendTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CAN_sendTask_callback */
void CAN_sendTask_callback(void *argument)
{
  /* USER CODE BEGIN CAN_sendTask_callback */
  MX_WWDG_Init(); // 窗口看门狗
  /* Infinite loop */
  for (;;)
  {
    if (Global.mode == LOCK)
    {
      CAN1_send_ZERO_current();
      CAN2_send_ZER0_current();
    } // 发送电机控制电流		}
    else
    {
      CAN1_send_current();
      CAN2_send_current();
    }
//    osDelay(5);
		osDelay(5);
    HAL_WWDG_Refresh(&hwwdg);
  }
  /* USER CODE END CAN_sendTask_callback */
}

/* USER CODE BEGIN Header_RemoteTask_callback */
/**
 * @brief Function implementing the RemoteTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RemoteTask_callback */
void RemoteTask_callback(void *argument)
{
  /* USER CODE BEGIN RemoteTask_callback */
  /* Infinite loop */
  for (;;)
  {
//    remote_control_task();

    osDelay(1);
  }
  /* USER CODE END RemoteTask_callback */
}

/* USER CODE BEGIN Header_ChassisTask_callback */
/**
 * @brief Function implementing the ChassisTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ChassisTask_callback */
void ChassisTask_callback(void *argument)
{
  /* USER CODE BEGIN ChassisTask_callback */

  /* Infinite loop */
  for (;;)
  {
    Chassic_course_solving(chassis.speed_RC.x,chassis.speed_RC.y,chassis.speed_RC.r);
		Chassis_course_updata();
		Chassis_course_pid_cal();
		Chassis_velocity_calc(chassis.speed_RC.x,chassis.speed_RC.y,chassis.speed_RC.r);
    osDelay(5);
  }
  /* USER CODE END ChassisTask_callback */
}

/* USER CODE BEGIN Header_GimbalTask_callback */
/**
 * @brief Function implementing the GimbalTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_GimbalTask_callback */
void GimbalTask_callback(void *argument)
{
  /* USER CODE BEGIN GimbalTask_callback */
  /* Infinite loop */

  for (;;)
  {
//				if(Get_sys_time_ms() - Time_delay_trs >1.0)
//		{
//				   dm4310_enable(&hcan2,&dm4310);
//				Time_delay_trs = Get_sys_time_ms();
//		}
		if(dm4310.para.state != 1)
		{
			dm4310_enable(&hcan2,&dm4310);
		}
		big_yaw_cal();
		relative_angle_big_yaw_send();
		
    osDelay(2);
//		osDelay(5);
  }
  /* USER CODE END GimbalTask_callback */
}

/* USER CODE BEGIN Header_ErrorDetectTask_callback */
/**
 * @brief Function implementing the ErrorDetectTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ErrorDetectTask_callback */
void ErrorDetectTask_callback(void *argument)
{
  /* USER CODE BEGIN ErrorDetectTask_callback */
  /* Infinite loop */
  for (;;)
  {
//    Error_detect_flush();
    osDelay(100);
  }
  /* USER CODE END ErrorDetectTask_callback */
}

/* USER CODE BEGIN Header_RefereeTask_callback */
/**
 * @brief 读取裁判系统相关信息，还有ui发送任务.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_RefereeTask_callback */
void RefereeTask_callback(void *argument)
{
  /* USER CODE BEGIN RefereeTask_callback */

  /* Infinite loop */
  for (;;)
  {
    referee_usart_task();
    Required_Data();
    Cap_Required_Data();
    cap_update();

    osDelay(10);
  }
  /* USER CODE END RefereeTask_callback */
}

/* USER CODE BEGIN Header_UI_Refresh_callback */
/**
 * @brief Function implementing the UI_Refresh thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UI_Refresh_callback */
void UI_Refresh_callback(void *argument)
{
  /* USER CODE BEGIN UI_Refresh_callback */
  /* Infinite loop */
  for (;;)
  {
//		extern float encoder_difference;
////		UploadData_vofa(get_motor_data(chassis_turn_BL).ecd,get_motor_data(chassis_turn_BL).last_ecd,encoder_difference+2000,0);
//		UploadData_vofa(encoder_difference,0,0,0);
		
    osDelay(1);
  }
  /* USER CODE END UI_Refresh_callback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

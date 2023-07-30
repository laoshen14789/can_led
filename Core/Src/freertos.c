/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "canNode.h"
#include <string.h>
#include "tim.h"
#include "flash.h"
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
ledStatus_s ledStatus_g = {0};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void led_status_change_handle(ledStatus_s *status);
void pwm_switch_handle(int value);
void status_led_switch_handle(int value);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    register_set_led_rgb_callback(led_status_change_handle);
    register_pwm_switch_callback(pwm_switch_handle);
    register_set_status_led_switch_callback(status_led_switch_handle);
    user_data_init();
    init_can_node();
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void led_status_change_handle(ledStatus_s *status)
{
    // memcpy(&ledStatus_g, status, sizeof(ledStatus_s));
    // osSemaphoreRelease(ledSemHandle);
    switch (status->ledNum)
    {
    case 0:
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, status->ledColor.green * 2);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, status->ledColor.red * 7);
        __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_3, status->ledColor.blue * 7);
        break;
    case 1:
        // map PWM to -1 to 1, assuming 1500 trim. If the servo has natural PWM
        // support then we should use it directly instead

        break;
    }
}

void pwm_switch_handle(int value)
{
    if(value == 1)
    {
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    }
    else
    {
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);
    }
}

void status_led_switch_handle(int value)
{
    if(value == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, SET);
    }
}

/* USER CODE END Application */


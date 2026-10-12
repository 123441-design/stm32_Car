/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "remote.h"
#include "control.h"
#include "encoder.h"
#include "car_encoder.h"

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

volatile uint32_t remote_stack_free_words = 0;
volatile uint32_t encoder_stack_free_words = 0;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId remoteTaskHandle;
osThreadId encoder_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartRemoteTask(void const * argument);
void Start_encoder_task(void const * argument);

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

  /* definition and creation of remoteTask */
  osThreadDef(remoteTask, StartRemoteTask, osPriorityNormal, 0, 128);
  remoteTaskHandle = osThreadCreate(osThread(remoteTask), NULL);

  /* definition and creation of encoder_task */
  osThreadDef(encoder_task, Start_encoder_task, osPriorityNormal, 0, 128);
  encoder_taskHandle = osThreadCreate(osThread(encoder_task), NULL);

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
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartRemoteTask */
/**
* @brief Function implementing the remoteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRemoteTask */
void StartRemoteTask(void const * argument)
{
  /* USER CODE BEGIN StartRemoteTask */
  /* Infinite loop */
  
 TickType_t last_check = xTaskGetTickCount();

  for (;;)
    {
        remote_poll();
        car_control_update(car_remote);

        // 计算任务栈剩余空间，便于 Keil Watch 查看
        if ((xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(1000))
        {
            last_check = xTaskGetTickCount();
            remote_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
        }

        osDelay(1);
    }

   

  /* USER CODE END StartRemoteTask */
}

/* USER CODE BEGIN Header_Start_encoder_task */
/**
* @brief Function implementing the encoder_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_encoder_task */
void Start_encoder_task(void const * argument)
{
  /* USER CODE BEGIN Start_encoder_task */
  TickType_t last_wake;
  TickType_t last_stack_check;
  int32_t counts[CAR_WHEEL_COUNT];
  float speeds_mps[CAR_WHEEL_COUNT];
  int i;

  /* 从启动编码器后开始计时，首次采样等待完整的 20 ms。 */
  encoder_start();
  last_wake = xTaskGetTickCount();
  last_stack_check = last_wake;

  for (;;)
  {
    //last_wake会自动更新为上次唤醒时间，保证周期性执行
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20U));

    /* 暂按示例工程的电机/编码器对应关系与符号；需逐轮实测确认。 */
    counts[CAR_WHEEL_FRONT_LEFT]  = -read_encoder_count(&htim8);
    counts[CAR_WHEEL_FRONT_RIGHT] = -read_encoder_count(&htim4);
    counts[CAR_WHEEL_REAR_LEFT]   =  read_encoder_count(&htim1);
    counts[CAR_WHEEL_REAR_RIGHT]  =  read_encoder_count(&htim3);

    car_encoder_calculate_wheel_speeds(counts, speeds_mps);

    for (i = 0; i < CAR_WHEEL_COUNT; ++i)
    {
      encoder_wheel_counts[i] = counts[i];
      encoder_wheel_speed_mps[i] = speeds_mps[i];
    }


    //计算任务栈剩余空间，便于 Keil Watch 查看
    if ((TickType_t)(xTaskGetTickCount() - last_stack_check) >= pdMS_TO_TICKS(1000U))
    {
      last_stack_check = xTaskGetTickCount();
      encoder_stack_free_words = uxTaskGetStackHighWaterMark(NULL);
    }
  }
  /* USER CODE END Start_encoder_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */


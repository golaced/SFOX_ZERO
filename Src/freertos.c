/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "stm32f4xx_hal.h"
#include "app_ano.h"
#include "mpu9250.h"
#include "ms5803.h"
#include "motor_pwm.h"
#include "app_led.h"
// #include "app_ins.h"
#include "time.h"
#include "app_ctrl.h"
#include "app_ros.h"
#include "app_uwb.h"
#include "app_ins_ekf_quaternion.h"
#include "gps_m8n.h"
/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId myTask02NormalHandle;
osThreadId myTask01RealTimHandle;
osMessageQId myQueue02GPSM8NToInsHandle;
osMessageQId myQueue01MPU9250ToANOHandle;
osMessageQId myQueue03MPU9250ToInsHandle;
osSemaphoreId myBinarySem01MPU9250GyroAccCalibrateOffsetHandle;
osSemaphoreId myBinarySem02LED1ONHandle;
osSemaphoreId myBinarySem03LED2ONHandle;
osSemaphoreId myBinarySem05LED1PulsateHandle;
osSemaphoreId myBinarySem06LED2PulsateHandle;
osSemaphoreId myBinarySem04MPU9250MagCalibrateHandle;
osSemaphoreId myBinarySem07GPSUpdateHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void StartTask02Normal(void const * argument);
void StartTask01RealTime(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
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

/* USER CODE BEGIN 3 */
__weak void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySem01MPU9250GyroAccCalibrateOffset */
  osSemaphoreDef(myBinarySem01MPU9250GyroAccCalibrateOffset);
  myBinarySem01MPU9250GyroAccCalibrateOffsetHandle = osSemaphoreCreate(osSemaphore(myBinarySem01MPU9250GyroAccCalibrateOffset), 1);

  /* definition and creation of myBinarySem02LED1ON */
  osSemaphoreDef(myBinarySem02LED1ON);
  myBinarySem02LED1ONHandle = osSemaphoreCreate(osSemaphore(myBinarySem02LED1ON), 1);

  /* definition and creation of myBinarySem03LED2ON */
  osSemaphoreDef(myBinarySem03LED2ON);
  myBinarySem03LED2ONHandle = osSemaphoreCreate(osSemaphore(myBinarySem03LED2ON), 1);

  /* definition and creation of myBinarySem05LED1Pulsate */
  osSemaphoreDef(myBinarySem05LED1Pulsate);
  myBinarySem05LED1PulsateHandle = osSemaphoreCreate(osSemaphore(myBinarySem05LED1Pulsate), 1);

  /* definition and creation of myBinarySem06LED2Pulsate */
  osSemaphoreDef(myBinarySem06LED2Pulsate);
  myBinarySem06LED2PulsateHandle = osSemaphoreCreate(osSemaphore(myBinarySem06LED2Pulsate), 1);

  /* definition and creation of myBinarySem04MPU9250MagCalibrate */
  osSemaphoreDef(myBinarySem04MPU9250MagCalibrate);
  myBinarySem04MPU9250MagCalibrateHandle = osSemaphoreCreate(osSemaphore(myBinarySem04MPU9250MagCalibrate), 1);

  /* definition and creation of myBinarySem07GPSUpdate */
  osSemaphoreDef(myBinarySem07GPSUpdate);
  myBinarySem07GPSUpdateHandle = osSemaphoreCreate(osSemaphore(myBinarySem07GPSUpdate), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  osSemaphoreWait(myBinarySem01MPU9250GyroAccCalibrateOffsetHandle,0);
  osSemaphoreWait(myBinarySem02LED1ONHandle,0);
  osSemaphoreWait(myBinarySem03LED2ONHandle,0);
  osSemaphoreWait(myBinarySem04MPU9250MagCalibrateHandle,0);
  osSemaphoreWait(myBinarySem05LED1PulsateHandle,0);
  osSemaphoreWait(myBinarySem05LED1PulsateHandle,0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02Normal */
  osThreadDef(myTask02Normal, StartTask02Normal, osPriorityNormal, 0, 1024);
  myTask02NormalHandle = osThreadCreate(osThread(myTask02Normal), NULL);

  /* definition and creation of myTask01RealTim */
  osThreadDef(myTask01RealTim, StartTask01RealTime, osPriorityRealtime, 0, 2048);
  myTask01RealTimHandle = osThreadCreate(osThread(myTask01RealTim), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of myQueue02GPSM8NToIns */
/* what about the sizeof here??? cd native code */
  osMessageQDef(myQueue02GPSM8NToIns, 1, uint32_t);
  myQueue02GPSM8NToInsHandle = osMessageCreate(osMessageQ(myQueue02GPSM8NToIns), NULL);

  /* definition and creation of myQueue01MPU9250ToANO */
/* what about the sizeof here??? cd native code */
  osMessageQDef(myQueue01MPU9250ToANO, 1, uint32_t);
  myQueue01MPU9250ToANOHandle = osMessageCreate(osMessageQ(myQueue01MPU9250ToANO), NULL);

  /* definition and creation of myQueue03MPU9250ToIns */
/* what about the sizeof here??? cd native code */
  osMessageQDef(myQueue03MPU9250ToIns, 1, uint32_t);
  myQueue03MPU9250ToInsHandle = osMessageCreate(osMessageQ(myQueue03MPU9250ToIns), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    //osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* StartTask02Normal function */
void StartTask02Normal(void const * argument)
{
  /* USER CODE BEGIN StartTask02Normal */
  /* Infinite loop */
  for(;;)
  {
    //
    ano_process(0);
    //
    osDelay(20);
  }
  /* USER CODE END StartTask02Normal */
}

/* StartTask01RealTime function */
void StartTask01RealTime(void const * argument)
{
  /* USER CODE BEGIN StartTask01RealTime */
  /* Infinite loop */
  for(;;)
  {
    //

    //
    MPU9250_process();
    //
    app_ins_ekf_quaternion_thread(float dt_s);
    //
    app_ctrl_thread(float dT);
    //

    osDelay(10);
  }
  /* USER CODE END StartTask01RealTime */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

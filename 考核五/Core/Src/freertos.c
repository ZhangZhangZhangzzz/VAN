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
#include "usart.h"
#include "stdio.h"
#include "kalman.h"
#include "mpu6050.h"
#include "bsp_can.h"
#include "CAN_receive.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*********************************DMA tast*********************************************/
#define RX_DATA_MAX 256   //接受数据最大值
uint16_t i;
char ReceiveData[RX_DATA_MAX];
/*********************************IMU tast*********************************************/
float gyro[3]={0},accel[3]={0} ,temperate=0 ;
/*********************************GM6020 tast*********************************************/
char   flag_buff[1];//锟斤拷志位锟斤拷锟捷伙拷锟斤拷锟斤拷
uint8_t   flag = 0 ; //0: speed  1:angle
int16_t   cnt = 0;
int16_t	  deviation_angle = 180 ;//锟角讹拷偏锟斤拷锟斤拷
int16_t	  deviation_speed = 200 ;//锟劫讹拷偏锟斤拷锟斤拷
int16_t  target_yaw_speed = 40;//目锟斤拷锟劫讹拷
float     target_yaw_angle = 40;//目锟斤拷嵌锟�
float     now_yaw_angle;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId UART_DMAHandle;
osThreadId IMU_StartHandle;
osThreadId CAN_GM6050Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

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
  /* definition and creation of UART_DMA */
  osThreadDef(UART_DMA, StartDefaultTask, osPriorityNormal, 0, 128);
  UART_DMAHandle = osThreadCreate(osThread(UART_DMA), NULL);

  /* definition and creation of IMU_Start */
  osThreadDef(IMU_Start, StartTask02, osPriorityIdle, 0, 512);
  IMU_StartHandle = osThreadCreate(osThread(IMU_Start), NULL);

  /* definition and creation of CAN_GM6050 */
  osThreadDef(CAN_GM6050, StartTask03, osPriorityIdle, 0, 2048);
  CAN_GM6050Handle = osThreadCreate(osThread(CAN_GM6050), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the UART_DMA thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,(uint8_t *) ReceiveData, sizeof(ReceiveData));//开启DMA
	 __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);//关闭 DMA半完成通道，以防数据传输一半就进入中断回调函数
  /* Infinite loop */
  for(;;)
  {


	  //TASK1:DMA_UART


    osDelay(5);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the IMU_Start thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */


  /* Infinite loop */
  for(;;)
  {
	  //TASK2:MPU6050
	  Angle_Calcu();
	  osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the CAN_GM6050 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {

	  	  now_yaw_angle = (float)(motor_chassis[0].ecd - 0)/8191.0f * 360.0f;

	  	  cnt++;
	  	  if(cnt >= 100)
	  	  {
	  		  cnt = 0 ;
	  		  target_yaw_angle += (float)deviation_angle;
	  		  target_yaw_speed += deviation_speed;
	  		  deviation_speed = - deviation_speed ;
	  		  deviation_angle = - deviation_angle ;

	  	  }

	  	  if(flag == 0)
	  	  {
	  		  pid_calc(&gimbal_yaw_speed_pid,target_yaw_speed, (motor_chassis[0]).speed_rpm);
	  		  CAN_cmd_gimbal(gimbal_yaw_speed_pid.output,0, 0, 0);//
	  		  printf(" %f, %d ,%d \r\n",gimbal_yaw_speed_pid.output,(motor_chassis[0]).speed_rpm,target_yaw_speed);
	  	  }
	  	  else
	  	  {
	  		 pid_calc(&gimbal_yaw_angle_pid,target_yaw_angle, now_yaw_angle);
	  		 CAN_cmd_gimbal(gimbal_yaw_angle_pid.output,0, 0, 0);
	  		 printf("%f ,%f , %f \r\n",gimbal_yaw_angle_pid.output,now_yaw_angle,target_yaw_angle);
	  	  }



	  	  	  printf("ecd,speed_rpm,given_current,temperate:%d,%d,%d,%d \r\n", (motor_chassis[0]).ecd, (motor_chassis[0]).speed_rpm, (motor_chassis[0]).given_current, (motor_chassis[0]).temperate);
    osDelay(20);
  }
  /* USER CODE END StartTask03 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

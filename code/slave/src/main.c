/*******************************************************************************
  * @file    main.c
  * @author  Diego Macario
	* @version V1.0.0
  * @date    31-March-2016
  * @brief   This file corresponds to the slave that provides acceleration and
  *	         temperature readings
  ******************************************************************************
  */

/*
   _____ _                 
  / ____| |                
 | (___ | | __ ___   _____ 
  \___ \| |/ _` \ \ / / _ \
  ____) | | (_| |\ V /  __/
 |_____/|_|\__,_| \_/ \___|
                           	
*/

#include "stm32f4xx_hal.h"              
#include "cmsis_os.h"                   
#include "RTE_Components.h"             

/* Accelerometer initialization functions */
extern void Initialize_Accelerometer(void);
extern void start_Thread_Accelerometer  (void);
extern void Thread_Accelerometer(void const *argument);
extern osThreadId tid_Thread_Accelerometer;

/* Temperature sensor initialization functions */
extern void Initialize_Temperature_Sensor(void);
extern void start_Thread_Temperature_Sensor  (void);
extern void Thread_Temperature_Sensor(void const *argument);
extern osThreadId tid_Thread_Temperature_Sensor;

extern int start_Thread_LED(void);

/* LED initialization functions */
extern int start_Thread_LED  (void);
extern void Thread_LED(void const *argument);
extern osThreadId tid_Thread_LED;

/* Communication initialization functions */
extern void Initialize_Communication(void);
extern void start_Thread_Communication  (void);
extern void Thread_Communication(void const *argument);
extern osThreadId tid_Thread_Communication;

/**
	These lines are mandatory to make CMSIS-RTOS RTX work with te new Cube HAL
*/
#ifdef RTE_CMSIS_RTOS_RTX
extern uint32_t os_time;

uint32_t HAL_GetTick(void) { 
  return os_time; 
}
#endif

/**
  * System Clock Configuration
  */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the
     device is clocked below the maximum system frequency (see datasheet). */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

/**
  * Main function
  */
int main (void) {

  osKernelInitialize();                     /* initialize CMSIS-RTOS            */

  HAL_Init();                               /* Initialize the HAL Library       */

  SystemClock_Config();                     /* Configure the System Clock       */
	
	Initialize_Accelerometer();               /* Initialize accelerometer         */
	start_Thread_Accelerometer();             /* Create accelerometer thread      */
	
	Initialize_Temperature_Sensor();          /* Initialize accelerometer         */
	start_Thread_Temperature_Sensor();        /* Create accelerometer thread      */
	
	start_Thread_LED();                       /* Initialize and create LED thread */
	
	Initialize_Communication();               /* Initialize communication         */
	start_Thread_Communication();             /* Create communication thread      */
  
	osKernelStart();                          /* Start thread execution           */
}

/*******************************************************************************
  * @file    Thread_LED.c
  * @author  Saki Kajita and Stephen Cambridge
	* @version V1.0.0
  * @date    12-April-2016
  * @brief   This file controls the LED display
  ******************************************************************************
  */
	
/*

  _      ______ _____    _____  _           _             
 | |    |  ____|  __ \  |  __ \(_)         | |            
 | |    | |__  | |  | | | |  | |_ ___ _ __ | | __ _ _   _ 
 | |    |  __| | |  | | | |  | | / __| '_ \| |/ _` | | | |
 | |____| |____| |__| | | |__| | \__ \ |_) | | (_| | |_| |
 |______|______|_____/  |_____/|_|___/ .__/|_|\__,_|\__, |
                                     | |             __/ |
                                     |_|            |___/ 

*/
	
#include "cmsis_os.h"                  
#include "stm32f4xx_hal.h"
#include <math.h>
#include "stm32f4xx_hal_tim.h"

void Thread_LED (void const *argument);                
osThreadId tid_Thread_LED;                              
osThreadDef(Thread_LED, osPriorityNormal, 1, 0);

/*----------------------------------------------------------------------------
 *      Prototypes
 *---------------------------------------------------------------------------*/

void PWM_Config(void);
void LED_delay(int delay);
void LED_display(int pattern, int frequency);
void LED_Off(void);
void LED_4_On (int frequency);
void LED_Rotate(int frequency);
void LED_Custom (int blink_order, int speed);
void Custom_PWM (int frequency);

/*----------------------------------------------------------------------------
 *      Variables and flags
 *---------------------------------------------------------------------------*/
 
/* --- Initialization structs ---------------------------------------------- */

GPIO_InitTypeDef 	LED_configuration;
TIM_HandleTypeDef TimHandle;
TIM_OC_InitTypeDef OCInit;

/* --- Internal variables -------------------------------------------------- */

int LED_flag = 1;

/*****************************************************************************
 * Shared Variables                                                          *
 *****************************************************************************/

int LED_Frequency;                  /* PWM frequency provided by the master
									   - This variables is shared with Thread_Communication
									   - It is only read inside Thread_LED */

int LED_Pattern;                    /* Flag that the determines the pattern displayed by the LED's
									   - This variables is shared with Thread_Communication
									   - It is only read inside Thread_LED */
																		
extern int timerCount;              /* Used by LED_Thread to control the LED display based on the 10 ms period of TIM4
									   - This variable is shared with Thread_Temperature_Sensor
									   - It is only read by Thread_LED */

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/
 
int start_Thread_LED (void) {
	
	LED_Frequency = 0;
	LED_Pattern = 0;   // LED's initially turned off
	
    tid_Thread_LED = osThreadCreate(osThread(Thread_LED ), NULL); // Start LED_Thread
    if (!tid_Thread_LED) return(-1); 
    return(0);
}

/*----------------------------------------------------------------------------
 *      Thread  'LED_Thread': Toggles LED
 *---------------------------------------------------------------------------*/

void Thread_LED (void const *argument) {
	
	while(1) {
		
		LED_display(LED_Pattern, LED_Frequency);

	}
	
}

/*----------------------------------------------------------------------------
 *      LED functions
 *---------------------------------------------------------------------------*/

/* --- Configuration functions --------------------------------------------- */

/**
  * @brief  This function initializes the LED display pins
  * @param  None
  * @retval None
  */
void initializeLED_IO (void) {
	
	__HAL_RCC_GPIOD_CLK_ENABLE();
	
	LED_configuration.Pin	= GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	LED_configuration.Mode 	= GPIO_MODE_OUTPUT_PP;
	LED_configuration.Speed	= GPIO_SPEED_FREQ_VERY_HIGH;
	LED_configuration.Pull	= GPIO_NOPULL;
	HAL_GPIO_Init(GPIOD, &LED_configuration);
	
}

/**
  * @brief  This function configures the PWM timer
  * @param  None
  * @retval None
  */
void PWM_Config(void) {
	
	TIM_Base_InitTypeDef TimInit;
	
	/* Initialize the timer */
	__HAL_RCC_TIM4_CLK_ENABLE();
		
	TimInit.Prescaler = 0;
	TimInit.Period = 8399;
	TimInit.ClockDivision= TIM_CLOCKDIVISION_DIV1;
	TimInit.CounterMode = TIM_COUNTERMODE_UP;
		
	TimHandle.Instance = TIM4;
	TimHandle.Init = TimInit;
	TimHandle.State = HAL_TIM_STATE_RESET;
		
	/* Set the timer */
	HAL_TIM_OC_Init(&TimHandle);
		
	OCInit.OCMode = TIM_OCMODE_PWM1;
	OCInit.OCPolarity = TIM_OCPOLARITY_HIGH;
		
	__HAL_RCC_GPIOD_CLK_ENABLE();

}

/* --- Display functions --------------------------------------------------- */

/**
  * @brief  This function defines the LED pattern and PWM frequency
  * @param  pattern: Specifies the desire pattern
  * @param  frequency: Specifies the PWM frequency
  * @retval None
  */
void LED_display(int pattern, int frequency) {
	
	switch (pattern){
		
		case 1 :                        // All LED's turned on
			PWM_Config();
			LED_4_On(frequency);
			break;
		case 2 :                        // LED's rotating
			initializeLED_IO();
			LED_Rotate (frequency);
			break;
		case 3 :                        // LED's follow custom pattern 
			initializeLED_IO();    
			LED_Custom (1342, frequency);
		case 4 :                        // LED's with PWM
			Custom_PWM (frequency); 
		default:
			LED_Off ();
		
	}
	
}

/**
  * @brief  This function is used to turn all the LED's off
  * @param  None
  * @retval None
  */
void LED_Off(void) {
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	
	osDelay(250); // When the LED's are turned off, put the thread to sleep for 25 ms and then check for new conditions
	
}

/**
  * @brief  This function is used to turn all the LED's on with PWM
  * @param  frequency: PWM frequency
  * @retval None
  */
void LED_4_On (int frequency) {
	
	int intensity = 83 * frequency + 97; 
	
	LED_configuration.Mode  = GPIO_MODE_AF_PP;
	LED_configuration.Pull  = GPIO_NOPULL;
	LED_configuration.Speed = GPIO_SPEED_FREQ_HIGH;
	LED_configuration.Alternate = GPIO_AF2_TIM4;
	LED_configuration.Pin =  GPIO_PIN_12 | GPIO_PIN_13| GPIO_PIN_14| GPIO_PIN_15 ;
	
	HAL_GPIO_Init(GPIOD, &LED_configuration);
	
	OCInit.Pulse = intensity;
	
	HAL_TIM_OC_ConfigChannel(&TimHandle, &OCInit, TIM_CHANNEL_4);
	HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_4);
	
	HAL_TIM_OC_ConfigChannel(&TimHandle, &OCInit, TIM_CHANNEL_3);
	HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_3);
	
	HAL_TIM_OC_ConfigChannel(&TimHandle, &OCInit, TIM_CHANNEL_2);
	HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_2);
	
	HAL_TIM_OC_ConfigChannel(&TimHandle, &OCInit, TIM_CHANNEL_1);
	HAL_TIM_OC_Start(&TimHandle, TIM_CHANNEL_1);
	
}

/**
  * @brief  This function is used to make the LED's rotate clockwise
  * @param  frequency: PWM frequency
  * @retval None
  */
void LED_Rotate (int frequency) {
	
	int speed_timer = (11 - abs(frequency)) * 5;
	
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);	
	
	while(LED_Pattern == 2 && frequency == LED_Frequency) {
		
		if(frequency >= 0) {
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

			LED_delay(speed_timer);
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			
			LED_delay(speed_timer);

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			
			LED_delay(speed_timer);
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			
			LED_delay(speed_timer);
			
		} 
		
		else {
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

			LED_delay(speed_timer);
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
			
			LED_delay(speed_timer);

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			
			LED_delay(speed_timer);
			
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			
			LED_delay(speed_timer);
			
		}
	}
}

/**
  * @brief  This function is used to make the LED's follow a custom pattern
  * @param  blink_order: Defines the order in which the LED's spin
  * @param  speed: Defines the speed of rotation
  * @retavl None
  */
void LED_Custom (int blink_order, int speed) {
	
	int digit_mod = 0;
	int digit;
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);	
	
	while(LED_Pattern == 3 && speed == LED_Frequency) {
				
		switch(digit_mod) {
			case(0):
				digit = blink_order / 1000;
				if(speed>=0) {digit_mod++;} else {digit_mod = 3;}
				break;
			case(1):
				digit = (blink_order / 100) % 10;
				if(speed>=0) {digit_mod++;} else {digit_mod--;}
				break;
			case(2):
				digit = (blink_order / 10) % 10;
				if(speed>=0) {digit_mod++;} else {digit_mod--;}
				break;
			case(3):
				digit = blink_order % 10;
				if(speed>=0) {digit_mod = 0;} else {digit_mod--;}
				break;
			default:
				break;	
		}
		
		switch(digit) {
	
			case(1) :
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
			
			case(2) :		
		
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
				break;	
		
			case(3) :
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
				break;
		
			case(4) :
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
				break;	
			
			default :
				break;
		}
		
		LED_delay((10 - speed)*5);
		
	}
}

/**
  * @brief  This function is used to make the LED's follow a custom pattern
  * @param  frequency: PWM frequency
  * @retval None
  */
void Custom_PWM (int frequency) {
	
	int speed_timer = (11 - abs(frequency)) * 5;
	int intensity = frequency;
	int rising = 1;
	PWM_Config();

	while(LED_Pattern == 4 && frequency == LED_Frequency){

		if(rising){
			intensity++;
			if(intensity > 10) {
				intensity = 10;
				rising = 0;
			}
		} 
		else {
			intensity--;
			if(intensity < 1) {
				intensity = 1;
				rising = 1;
			}
		}
		LED_4_On(intensity);
		LED_delay(speed_timer);
	}
	
}

/**
  * @brief  This function is used to make Thread_LED sleep
  * @param  delay: Length of the delay
  * @retval None
  */
void LED_delay(int delay) {
	osDelay(10 * delay);
}

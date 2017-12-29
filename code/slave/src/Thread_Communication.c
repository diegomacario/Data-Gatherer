/*******************************************************************************
  * @file    Thread_Accelerometer.c
  * @author  Diego Macario
  * @version V4.2.0
  * @date    12-April-2016
  * @brief   This file manages the SPI communication between the Discovery board
  *          (slave) and the Nucleo Board (master)
  ******************************************************************************
  */
	
/*       
   _____                                      _           _   _               _    _       _     
  / ____|                                    (_)         | | (_)             | |  | |     | |    
 | |     ___  _ __ ___  _ __ ___  _   _ _ __  _  ___ __ _| |_ _  ___  _ __   | |__| |_   _| |__  
 | |    / _ \| '_ ` _ \| '_ ` _ \| | | | '_ \| |/ __/ _` | __| |/ _ \| '_ \  |  __  | | | | '_ \ 
 | |___| (_) | | | | | | | | | | | |_| | | | | | (_| (_| | |_| | (_) | | | | | |  | | |_| | |_) |
  \_____\___/|_| |_| |_|_| |_| |_|\__,_|_| |_|_|\___\__,_|\__|_|\___/|_| |_| |_|  |_|\__,_|_.__/ 
                                                                                                                                                                                                                                                       
*/
	
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>

void Thread_Communication (void const *argument);         
osThreadId tid_Thread_Communication;                     
osThreadDef(Thread_Communication, osPriorityHigh, 1, 0);

/*----------------------------------------------------------------------------
 *      Prototypes
 *---------------------------------------------------------------------------*/
 
void SPI2_Init(void);
void Configure_I_Am_Ready_To_Send_You_What_You_Want_Pin(void);
void Configure_Master_Wants_Something_Interrupt_Pin(void);
void Configure_Receive_Instruction_Pins(void);
void Configure_Double_Tap_Pin(void);

void Initialize_Communication(void);

void Receive_Instruction(void);
void Transmit_Data(void);

int concatenate_instruction(int a, int b, int c, int d, int e);
void float2Bytes(float val, unsigned char* bytes_array);

/*----------------------------------------------------------------------------
 *      Variables and flags
 *---------------------------------------------------------------------------*/
 
/* --- Initialization structs ---------------------------------------------- */

SPI_HandleTypeDef SPI2_Handle;      /* SPI2 is used by the slave (Discovery board) to communicate with the master (Nucleo board) */

/* --- Internal variables -------------------------------------------------- */

int master_instruction;             /* Instruction code received from the master */

int Data_Mode_Flag;                 /* Defines what to send to the master based on the master instruction
								       - Equal to 1 when the master wants temperature
									   - Equal to 2 when the master wants the pitch
									   - Equal to 3 when the master wants the roll */

/*****************************************************************************
 * Shared Variables                                                          *
 *****************************************************************************/
 
extern float measured_temperature;  /* Temperature in Celcius 
									   - This variable is shared with Thread_Temperature_Sensor 
									   - It is only read inside Thread_Communication */
															 
extern double pitch;                /* Pitch measured by the accelerometer
									   - This variable is shared with Thread_Accelerometer
									   - It is only read inside Thread_Communication */
															
extern double roll;                 /* Roll measured by the accelerometer
									   - This variable is shared with Thread_Accelerometer
									   - It is only read inside Thread_Communication */

extern int LED_Frequency;			/* PWM frequency provided by the master
									   - This variables is shared with Thread_LED
									   - It is only written inside Thread_Communication */

extern int LED_Pattern;             /* Flag that the determines the pattern displayed by the LED's
									   - This variables is shared with Thread_LED
									   - It is only written inside Thread_Communication */

int master_asked;
/*****************************************************************************
 * Mutexes                                                                   *
 *****************************************************************************/
															 
extern osMutexId Measured_Temperature_Mutex;  /* Used to safely access the measured_temperature, which is shared by Thread_Temperature_Sensor and Thread_Communication */
extern osMutexId Tilt_Mutex;                  /* Used to safely access the pitch and roll, which are shared by Thread_Accelerometer and Thread_Communication */

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/

int start_Thread_Communication (void) {
	
  tid_Thread_Communication = osThreadCreate(osThread(Thread_Communication), NULL);
  if (!tid_Thread_Communication) return(-1); 
  return(0);
	
}

/*----------------------------------------------------------------------------
 *      Thread  'Communication_Thread': Sustains master/slave communication
 *---------------------------------------------------------------------------*/

void Thread_Communication (void const *argument) {
	
	while(1){
		
		// Wait until the master triggers an interrupt through pin E2
		osSignalWait(0x01, osWaitForever);
		
		master_asked++;

		Receive_Instruction();

		// Tell the master you want to talk
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_SET);

		Transmit_Data();
			
		// Done talking
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, GPIO_PIN_RESET);
	
	}
	
}

/*----------------------------------------------------------------------------
 *      Master/slave communication functions
 *---------------------------------------------------------------------------*/

/* --- Configuration functions --------------------------------------------- */

/**
  * @brief  This function configures the SPI2
  * @param  None
  * @retval None
  */
void SPI2_Init(void) {

	__HAL_RCC_SPI2_CLK_ENABLE();
	
    HAL_SPI_DeInit(&SPI2_Handle);
    SPI2_Handle.Instance                = SPI2;
	
	SPI2_Handle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_256;
	
    SPI2_Handle.Init.Direction 	        = SPI_DIRECTION_2LINES;
    SPI2_Handle.Init.CLKPhase 		      = SPI_PHASE_1EDGE;
    SPI2_Handle.Init.CLKPolarity 		    = SPI_POLARITY_LOW;
    SPI2_Handle.Init.CRCCalculation	    = SPI_CRCCALCULATION_DISABLED;
    SPI2_Handle.Init.CRCPolynomial 	    = 7;
    SPI2_Handle.Init.DataSize 		      = SPI_DATASIZE_8BIT;
    SPI2_Handle.Init.FirstBit 		      = SPI_FIRSTBIT_MSB;
	
    SPI2_Handle.Init.NSS 					      = SPI_NSS_SOFT;       
	
    SPI2_Handle.Init.TIMode 			      = SPI_TIMODE_DISABLED;
    SPI2_Handle.Init.Mode 				      = SPI_MODE_SLAVE;
	
	if (HAL_SPI_Init(&SPI2_Handle) != HAL_OK) {printf ("ERROR: Error in initialising SPI1 \n");};
  
	__HAL_SPI_ENABLE(&SPI2_Handle);
  
}

/**
  * @brief  This function configures the pin used to tell the master "I am ready to send you what you want"
  * @param  None
  * @retval None
  */
void Configure_I_Am_Ready_To_Send_You_What_You_Want_Pin(void) {
	
	GPIO_InitTypeDef GPIO_I_Sent_It_To_You_Pin;
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	/* Pin E7 is used by the slave to tell the master "I am ready to send you what you want" */
	GPIO_I_Sent_It_To_You_Pin.Pin = GPIO_PIN_7;  
    GPIO_I_Sent_It_To_You_Pin.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_I_Sent_It_To_You_Pin.Pull = GPIO_NOPULL;
    GPIO_I_Sent_It_To_You_Pin.Speed = GPIO_SPEED_FREQ_MEDIUM;

	HAL_GPIO_Init(GPIOE, &GPIO_I_Sent_It_To_You_Pin);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 | GPIO_PIN_6, GPIO_PIN_RESET);
	
}

/**
  * @brief  This function configures the pin used by the slave to know when the master asked it for something
  * @param  None
  * @retval None
  */
void Configure_Master_Wants_Something_Interrupt_Pin(void) {
	
	GPIO_InitTypeDef GPIO_Master_Wants_Something_Pin;
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	/* Pin E2 is used by the slave to know when the master asked it for something */
	GPIO_Master_Wants_Something_Pin.Pin = GPIO_PIN_2; 
    GPIO_Master_Wants_Something_Pin.Mode = GPIO_MODE_IT_RISING;
    GPIO_Master_Wants_Something_Pin.Pull = GPIO_PULLDOWN;
    GPIO_Master_Wants_Something_Pin.Speed = GPIO_SPEED_FREQ_MEDIUM;
	
	HAL_GPIO_Init(GPIOE, &GPIO_Master_Wants_Something_Pin);
	
	/* Priority of level 1 */
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);
	
}

/**
  * @brief  This function configures the pins used by the slave to receive instructions from the master
  * @param  None
  * @retval None
  */
void Configure_Receive_Instruction_Pins(void) {

	GPIO_InitTypeDef GPIO_Receive_Instruction_Pins;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	/* Configure pins used by the slave to receive instructions */
	GPIO_Receive_Instruction_Pins.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9; 
    GPIO_Receive_Instruction_Pins.Mode = GPIO_MODE_INPUT;
    GPIO_Receive_Instruction_Pins.Pull = GPIO_PULLDOWN;
    GPIO_Receive_Instruction_Pins.Speed = GPIO_SPEED_FREQ_MEDIUM;
	
	HAL_GPIO_Init(GPIOC, &GPIO_Receive_Instruction_Pins);
	
}

/**
  * @brief  This function configures the pin used by the slave to tell the master when a double occurs
  * @param  None
  * @retval None
  */
void Configure_Double_Tap_Pin(void) {

	GPIO_InitTypeDef GPIO_Double_Tap_Pin;
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	/* Pin E6 is used by the slave to tell the master "A double tap occurred" */
	GPIO_Double_Tap_Pin.Pin = GPIO_PIN_6;  
    GPIO_Double_Tap_Pin.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Double_Tap_Pin.Pull = GPIO_NOPULL;
    GPIO_Double_Tap_Pin.Speed = GPIO_SPEED_FREQ_MEDIUM;

	HAL_GPIO_Init(GPIOE, &GPIO_Double_Tap_Pin);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
	
}

 /* --- Total initialization function --------------------------------------- */

/**
   * @brief  A function used to configure and initialize the SPI2 and the necessary pins to establish communication between master and the slave
   * @param  None
   * @retval None
   */	
void Initialize_Communication(void) {
	
	master_asked = 0;
	
	/* Data/Interrupt pins */
	Configure_I_Am_Ready_To_Send_You_What_You_Want_Pin();
	Configure_Master_Wants_Something_Interrupt_Pin();
	Configure_Receive_Instruction_Pins();
	Configure_Double_Tap_Pin();
	
  /* SPI2 */
	SPI2_Init();
	
}

/**
   * @brief  A function used by the slave to receive instructions from the master
   * @param  None
   * @retval None
   */	
void Receive_Instruction(void) {

	int a, b, c, d, e;
	int instruction;
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_4) == 1) { // MSB
		e = 1;
	} else {
		e = 0;
	}
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == 1) {
		d = 1;
	} else {
		d = 0;
	}
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == 1) {
		c = 1;
	} else {
		c = 0;
	}
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == 1) {
		b = 1;
	} else {
		b = 0;
	}
	
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == 1) { // LSB
		a = 1;
	} else {
		a = 0;
	}
	
	instruction = concatenate_instruction(a, b, c, d, e);
	
	//  E  D  C  B  A
	// MSB         LSB
	
	switch(instruction) {
		
		/* PWM frequency */
		case 0:     // 0
			LED_Frequency = 0;
			break;
		case 1:     // 1
			LED_Frequency = 1;
			break;
		case 10:    // 2
			LED_Frequency = 2;
			break;
		case 11:    // 3
			LED_Frequency = 3;
			break;
		case 100:   // 4
			LED_Frequency = 4;
			break;
		case 101:   // 5
			LED_Frequency = 5;
			break;
		case 110:   // 6
			LED_Frequency = 6;
			break;
		case 111:   // 7
			LED_Frequency = 7;
			break;
		case 1000:  // 8
			LED_Frequency = 8;
			break;
		case 1001:  // 9
			LED_Frequency = 9;
			break;
		case 1010:  // 10
			LED_Frequency = 10;
			break;
		case 1011:  // 11
			LED_Frequency = -1;
			break;
		case 1100:  // 12
			LED_Frequency = -2;
			break;
		case 1101:  // 13
			LED_Frequency = -3;
			break;
		case 1110:  // 14
			LED_Frequency = -4;
			break;
		case 1111:  // 15
			LED_Frequency = -5;
			break;
		case 10000: // 16
			LED_Frequency = -6;
			break;
		case 10001: // 17
			LED_Frequency = -7;
			break;
		case 10010: // 18
			LED_Frequency = -8;
			break;
		case 10011: // 19
			LED_Frequency = -9;
			break;
		case 10100: // 20
			LED_Frequency = -10;
			break;
		
		/* LED Patterns */
		case 10101: // 21
			LED_Pattern = 0;    // Off
			break;
		case 10110: // 22
			LED_Pattern = 1;    // All on PWM
			break;
		case 10111: // 23
			LED_Pattern = 2;    // Rotating
			break;
		case 11000: // 24
			LED_Pattern = 3;    // Custom
			break;
		case 11001: // 25
			LED_Pattern = 4;    // Glow
			break;
		
		/* Return values requested by the board */
		case 11101: // 29
			Data_Mode_Flag = 3; // Roll
			break;
		case 11110: // 30         
			Data_Mode_Flag = 2; // Pitch
			break;
		case 11111: // 31       
      Data_Mode_Flag = 1; // Temperature
			break;
		default:
			break;
	}

}

/**
   * @brief  A function used by compose the instruction sent by the master
   * @param  a, b, c, d, e: Bits sent by the master. LSB is 'a' while MSB is 'e'
   * @retval Composed instruction
   */
int concatenate_instruction(int a, int b, int c, int d, int e) {

	  char my_array[5];
    
    // LSB (a)
    if (a == 0) {
        my_array[4] = '0';
    }
    else {
        my_array[4] = '1';
    }
    
    if (b == 0) {
        my_array[3] = '0';
    }
    else {
        my_array[3] = '1';
    }
    
    if (c == 0) {
        my_array[2] = '0';
    }
    else {
        my_array[2] = '1';
    }
    
    if (d == 0) {
        my_array[1] = '0';
    }
    else {
        my_array[1] = '1';
    }
    
    // MSB (e)
    if (e == 0) {
        my_array[0] = '0';
    }
    else {
        my_array[0] = '1';
    }
    
    return atoi(my_array);
	
}

/**
   * @brief  A function used by slave to transmit data to the master
   * @param  None
   * @retval None
   */
void Transmit_Data(void) {

	unsigned char bytes_array[4]; /* Bytes to be sent to the master */
	
	if (Data_Mode_Flag == 1) {
		float2Bytes(measured_temperature, bytes_array);
	}
	else if(Data_Mode_Flag == 2) {
		float2Bytes(pitch, bytes_array);
	}
	else if(Data_Mode_Flag == 3) {
		float2Bytes(roll, bytes_array);
	}
	
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_TXE) == RESET) {}
	SPI2_Handle.Instance->DR = bytes_array[0];
		
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_TXE) == RESET) {}
	SPI2_Handle.Instance->DR = bytes_array[1];
			
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_TXE) == RESET) {}
	SPI2_Handle.Instance->DR = bytes_array[2];
			
	while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_TXE) == RESET) {} 
	SPI2_Handle.Instance->DR = bytes_array[3];

}

/* --- Communication functions --------------------------------------------- */

/**
  * @brief  This function converts a float into 4 bytes and stores them in an array
  * @param  val: Float to be converted into 4 bytes
  * @param  bytes_array: Array used to store the 4 bytes produced by the function
  * @retval None
  */
void float2Bytes(float val, unsigned char* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    unsigned char temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

/* --- Handlers ------------------------------------------------------------ */

/**
  * @brief  This function handles EXTI2 interrupt requests
  * @param  None
  * @retval None
  */
void EXTI2_IRQHandler(void) {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

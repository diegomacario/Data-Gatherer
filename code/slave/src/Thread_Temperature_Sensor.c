/*******************************************************************************
  * @file    Thread_Temperature_Sensor.c
  * @author  Diego Macario
  * @version V1.0.0
  * @date    12-March-2016
  * @brief   This file manages the temperature sensor
  ******************************************************************************
  */
	
/*
  _______                                  _                     _____                           
 |__   __|                                | |                   / ____|                          
    | | ___ _ __ ___  _ __   ___ _ __ __ _| |_ _   _ _ __ ___  | (___   ___ _ __  ___  ___  _ __ 
    | |/ _ \ '_ ` _ \| '_ \ / _ \ '__/ _` | __| | | | '__/ _ \  \___ \ / _ \ '_ \/ __|/ _ \| '__|
    | |  __/ | | | | | |_) |  __/ | | (_| | |_| |_| | | |  __/  ____) |  __/ | | \__ \ (_) | |   
    |_|\___|_| |_| |_| .__/ \___|_|  \__,_|\__|\__,_|_|  \___| |_____/ \___|_| |_|___/\___/|_|   
                     | |                                                                         
                     |_|                                                                         

*/
	
#include "cmsis_os.h"      
#include "stm32f4xx_hal.h"

void Thread_Temperature_Sensor(void const *argument);            
osThreadId tid_Thread_Temperature_Sensor;                      
osThreadDef(Thread_Temperature_Sensor, osPriorityHigh, 1, 0);

typedef struct kalman_state {
	double q;  // Process noise covariance
	double r;  // Measurement noise covariance
	double x;  // Value
	double p;  // Estimation error covariance
	double k;  // Kalman gain
} kalman_state;

/*----------------------------------------------------------------------------
 *      Prototypes
 *---------------------------------------------------------------------------*/

void Configure_ADC(ADC_ChannelConfTypeDef ADC1_ChannelConf);
void Configure_TIM4(TIM_HandleTypeDef *TIM_Handle);
void Initialize_Temperature_Sensor(void);
float Convert_To_Celsius(int sample);
kalman_state *__init__ (double q, double r, double p, double k, double initial_value, kalman_state* kstate);
extern double Kalmanfilter_C(double measurement, kalman_state* kstate);

/*----------------------------------------------------------------------------
 *      Variables and flags
 *---------------------------------------------------------------------------*/

/* --- Initialization structs ---------------------------------------------- */

TIM_HandleTypeDef TIM4_Handle;            /* TIM3 struct */
ADC_HandleTypeDef ADC1_Handle;            /* ADC1 struct */
ADC_ChannelConfTypeDef ADC1_ChannelConf;  /* ADC1 Channel struct */

/* --- Kalman filter structs ----------------------------------------------- */

kalman_state *kstate;                     /* Temperature filter*/

/* --- Internal variables -------------------------------------------------- */

float raw_temperature;

/*****************************************************************************
 * Shared Variables                                                          *
 *****************************************************************************/
 
float measured_temperature;               /* Temperature in Celcius 
											 - This variable is shared with Thread_Communication
											 - It is only modified by Thread_Temperature_Sensor */
																						 
int timerCount;                           /* Used by LED_Thread to control the LED display based on the 10 ms period of TIM4
											 - This variable is shared with Thread_LED
											 - It is only modified by Thread_Temperature_Sensor */
																 
/*****************************************************************************
 * Mutexes                                                                   *
 *****************************************************************************/
																 
osMutexId Measured_Temperature_Mutex;     /* Used to safely access the measured_temperature, which is shared between the temperature sensor and the */
osMutexDef(Measured_Temperature_Mutex);  

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/

int start_Thread_Temperature_Sensor (void) {

	Measured_Temperature_Mutex = osMutexCreate(osMutex(Measured_Temperature_Mutex));
	
	tid_Thread_Temperature_Sensor = osThreadCreate(osThread(Thread_Temperature_Sensor), NULL);
    if (!tid_Thread_Temperature_Sensor) return(-1); 
    return(0);
	
}

/*----------------------------------------------------------------------------
 *  Thread  'Temperature_Sensor_Thread': Reads the temperature sensor values
 *---------------------------------------------------------------------------*/

void Thread_Temperature_Sensor (void const *argument) {
	
	while(1){
		
		osSignalWait(0x01, osWaitForever);
		
		// Filter and convert the new measurement
		osMutexWait(Measured_Temperature_Mutex, osWaitForever);
		measured_temperature = Convert_To_Celsius(raw_temperature);           // Convert measurement to Celsius
		measured_temperature = Kalmanfilter_C(measured_temperature, kstate);  // Filter the converted measurement
		osMutexRelease(Measured_Temperature_Mutex);
		
	}
	
}

/*----------------------------------------------------------------------------
 *      Temperature sensor functions
 *---------------------------------------------------------------------------*/
	
/* --- Configuration functions --------------------------------------------- */

/**
   * @brief  A function used to setup the ADC to sample Channel 16
   * @param  ADC1_ChannelConf: Configure the ADC to convert the values read by the temperature sensor that is hardwired to channel 16
   * @retval None
   */
void Configure_ADC(ADC_ChannelConfTypeDef ADC1_ChannelConf) {

	__HAL_RCC_ADC1_CLK_ENABLE();
	
	/* ADC_ChannelConfTypeDef struct */
	
	ADC1_ChannelConf.Channel = ADC_CHANNEL_16;                     // Channel 16 is hardwired to the temperature sensor
	ADC1_ChannelConf.Rank = 1;                                     
	ADC1_ChannelConf.SamplingTime = ADC_SAMPLETIME_480CYCLES;      
	ADC1_ChannelConf.Offset = 0;                                   // Reserved for future use
	
	/* ADC_HandleTypeDef struct */         
	
	ADC1_Handle.Instance = ADC1;
	ADC1_Handle.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;    
	ADC1_Handle.Init.Resolution = ADC_RESOLUTION_12B;              // Resolution of 12 bits to accurately encode measurements.
	ADC1_Handle.Init.DataAlign = ADC_DATAALIGN_RIGHT;              // Align each 12 bit measurement to the right.
	ADC1_Handle.Init.ScanConvMode = DISABLE;                       // Conversion performed in SINGLE mode instead of SCAN mode since we are working with one channel.
	ADC1_Handle.Init.EOCSelection = ADC_EOC_SINGLE_CONV;           // EOC flag is set at the end of single channel conversion. Each conversion data must be read.
	ADC1_Handle.Init.ContinuousConvMode = DISABLE;                 // Conversion performed in SINGLE mode instead of CONTINUOUS mode since we are working with one channel.
	ADC1_Handle.Init.DMAContinuousRequests = DISABLE;              // DMA requests performed in SINGLE mode instead of CONTINUOUS mode. We are not using DMA.
	ADC1_Handle.Init.NbrOfConversion = 1;                          // One conversion since we are working with one channel.
	ADC1_Handle.Init.DiscontinuousConvMode = DISABLE;              // Conversion is not performed in DISCONTINUOUS mode because we are using a regular channel.
	ADC1_Handle.Init.NbrOfDiscConversion = 1;                      // N/A
	ADC1_Handle.Init.ExternalTrigConv = ADC_SOFTWARE_START;        // Disable external triggers
	ADC1_Handle.Init.ExternalTrigConvEdge = ADC_SOFTWARE_START;    // Discard this parameter since we are not using an external trigger
	ADC1_Handle.NbrOfCurrentConversionRank = 1;
	
	HAL_ADC_Init(&ADC1_Handle);
	
	HAL_ADC_ConfigChannel(&ADC1_Handle, &ADC1_ChannelConf);

}
	
/**
   * @brief  A function used to configure the TIM3 timer
   * @param  TIM_Handle: Pointer to the timer configuration struct
   * @retval None
   */
void Configure_TIM4(TIM_HandleTypeDef *TIM_Handle) {
	
	__HAL_RCC_TIM4_CLK_ENABLE();
	
	/* 
	  | Prescaler | Frequency | Delay |  
	  |     84    |  1000 Hz  | 1 ms  |
	  |    168    |   500 Hz  | 2 ms  |
	  |    336    |   250 Hz  | 4 ms  | 
	  |    672    |   125 Hz  | 8 ms  |
	  |    840    |   100 Hz  | 10 ms | -> TIM4 is based on this delay
	*/
	
	TIM_Handle->Instance = TIM4;					   /* Register base address */
	TIM_Handle->Init.Prescaler = 840;                  /* Specifies the prescaler value used to divide the TIM clock */
    TIM_Handle->Init.CounterMode = TIM_COUNTERMODE_UP; /* Specifies the counter mode */
    TIM_Handle->Init.Period = 1000;                    /* Specifies the period value to be loaded into the active Auto-Reload Register at the next update event */
	
	HAL_TIM_Base_Init(TIM_Handle);
	HAL_TIM_Base_Start_IT(TIM_Handle);
	
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 2);             /* Priority of level 2 for the timer */
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	
}

/* --- Total initialization function --------------------------------------- */

/**
   * @brief  A function used to configure and initialize the temperature sensor
   * @param  None
   * @retval None
   */	
void Initialize_Temperature_Sensor(void) {

	timerCount = 0;
	Configure_TIM4(&TIM4_Handle);          /* Configure the timer */
	Configure_ADC(ADC1_ChannelConf);       /* Configure the ADC */
	
	kstate = __init__(0.0001, 0.5, 0.1, 0, 25, kstate);
	
	HAL_ADC_Start(&ADC1_Handle);
	
}

/* --- Temperature sensor functions ---------------------------------------- */

/**
   * @brief  A function used to convert the values obtained from the temperature sensor through the ADC to Celcius
   * @param  sample: Value obtained through the ADC from the temperature sensor
   * @retval Temperature in Celcius
   */
float Convert_To_Celsius(int sample) {
	
	float temperature = sample;
	
	// Old formula
	temperature = ((sample - 1087.228486646884) / 2.5) + 25;
	
	// New formula
	//temperature = (sample * (3.3f / 4096.0f));    //Reading in V
	//temperature = temperature - ((float) 0.76);   // Subtract the reference voltage at 25°C
	//temperature = temperature / ((float) 0.025);  // Divide by slope 2.5mV
	//temperature = temperature + 25;               // Add the 25°C
	
	return temperature; 

}

/* --- Handlers ------------------------------------------------------------ */

/**
  * @brief  This function handles TIM3 interrupt requests.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void) {
	
	HAL_TIM_IRQHandler(&TIM4_Handle);
	
}

/* --- Callbacks ----------------------------------------------------------- */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	
	if (htim == &TIM4_Handle) {
 
		timerCount = (timerCount + 1) % 100000;
		
		// If the ADC conversion is finished
		if (__HAL_ADC_GET_FLAG(&ADC1_Handle, ADC_FLAG_EOC) == 1) {

			// Read the temperature
			raw_temperature = HAL_ADC_GetValue(&ADC1_Handle);
			// Process the measurement by allowing the body of the thread loop to execute
			osSignalSet(tid_Thread_Temperature_Sensor, 0x01);
			// Start a new conversion
			HAL_ADC_Start(&ADC1_Handle);
			
		}
 
	}
	
}

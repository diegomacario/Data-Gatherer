/*******************************************************************************
  * @file    Thread_Accelerometer.c
  * @author  Diego Macario, Saki Kajita and Stephen Cambridge
  * @version V1.0.1
  * @date    14-April-2016
  * @brief   This file manages the accelerometer
  ******************************************************************************
  */
	
/*
                       _                               _            
     /\               | |                             | |           
    /  \   ___ ___ ___| | ___ _ __ ___  _ __ ___   ___| |_ ___ _ __ 
   / /\ \ / __/ __/ _ \ |/ _ \ '__/ _ \| '_ ` _ \ / _ \ __/ _ \ '__|
  / ____ \ (_| (_|  __/ |  __/ | | (_) | | | | | |  __/ ||  __/ |   
 /_/    \_\___\___\___|_|\___|_|  \___/|_| |_| |_|\___|\__\___|_|   
                                                                    
*/
	
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include "lis3dsh.h"
#include <math.h>
#include <stdlib.h>

# define M_PI 3.14159265358979323846

void Thread_Accelerometer (void const *argument);        
osThreadId tid_Thread_Accelerometer;                        
osThreadDef(Thread_Accelerometer, osPriorityHigh, 1, 0);

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

void Configure_LIS3DSH(LIS3DSH_InitTypeDef LIS3DSHStruct, LIS3DSH_DRYInterruptConfigTypeDef LIS3DSH_InterruptConfigStruct);
void Configure_LIS3DSH_Interrupt_Line(GPIO_InitTypeDef GPIO_E_Init_Accelerometer);
void Initialize_Accelerometer(void);
void Calculate_Tilt(kalman_state* kstate_x, kalman_state* kstate_y, kalman_state* kstate_z, double tilt_Array[2]);
kalman_state *__init__ (double q, double r, double p, double k, double initial_value, kalman_state* kstate);
double Kalmanfilter_C(double measurement, kalman_state* kstate);

void Detect_Double_Tap(double Z_Acceleration, double pitch, double roll);
int incr_counter(int src, int amt);
 
/*----------------------------------------------------------------------------
 *      Variables and flags
 *---------------------------------------------------------------------------*/
 
/* --- Initialization structs ---------------------------------------------- */

LIS3DSH_InitTypeDef LIS3DSHStruct;                                /* LIS3DSH struct */
LIS3DSH_DRYInterruptConfigTypeDef LIS3DSH_InterruptConfigStruct;  /* LIS3DSH Data ready interrupt struct */
GPIO_InitTypeDef GPIO_E_Init_Accelerometer;                       /* Port E - Pin 0 used as the interrupt line of the accelerometer */

/* --- Kalman filter structs ----------------------------------------------- */

kalman_state *kstate_x;     /* X acceleration filter */
kalman_state *kstate_y;     /* Y acceleration filter */
kalman_state *kstate_z;     /* Z acceleration filter */

/* --- Internal variables -------------------------------------------------- */

double tilt_Array[2];

/* Double tap */

int buffer_size = 100; // Buffer window with 100 measurements (spans 4 seconds worth of measurements)
int *acc_buf; 
int buf_counter = 0;       // Modulo counter for indexing circular acceleration buffer    
int i;                     // Generic loop index
double avg;                // Moving window avg
int sum = 0;               // Used in avg computation
int tap_latch;             // Time stamp of single tap, for purposes of debouncing
int tap_detected = 0;      // Debouncing flag

int Reset_Tap;             // Time stamp of single tap, for double tap resetting
int Tap_Counter;           // Index for FSM of double tap control logic
int Detected_Double_Tap_Flag;

/*****************************************************************************
 * Shared Variables                                                          *
 *****************************************************************************/
															 
double pitch;               /* Pitch measured by the accelerometer
							   - This variable is shared with Thread_Communication
							   - It is only modified by Thread_Accelerometer */
															
double roll;                /* Roll measured by the accelerometer
							   - This variable is shared with Thread_Communication
							   - It is only modified by Thread_Accelerometer */
															 
extern osThreadId tid_Thread_Communication; /* Thread ID of Thread_Communication
											   - Used inside HAL_GPIO_EXTI_Callback to trigger the operation of Thread_Communication when the master wants something */
															 
/*****************************************************************************
 * Mutexes                                                                   *
 *****************************************************************************/
															 
osMutexId Tilt_Mutex;    /* Used to safely access the pitch and roll, which are shared between the accelerometer and the */
osMutexDef(Tilt_Mutex);

/*----------------------------------------------------------------------------
 *      Create the thread within RTOS context
 *---------------------------------------------------------------------------*/

int start_Thread_Accelerometer (void) {
	
  Tilt_Mutex = osMutexCreate(osMutex(Tilt_Mutex));
	
  tid_Thread_Accelerometer = osThreadCreate(osThread(Thread_Accelerometer), NULL);
  if (!tid_Thread_Accelerometer) return(-1); 
  return(0);
	
}
/*----------------------------------------------------------------------------
 *      Thread  'Accelerometer_Thread': Reads accelerometer values
 *---------------------------------------------------------------------------*/

void Thread_Accelerometer (void const *argument) {
	
	while(1){
		
		// Execute the body of this loop every 25 ms
		osSignalWait(0x01, osWaitForever);
		
		// Read the x, y and z accelerations when they become available (every 4 ms)
		Calculate_Tilt(kstate_x, kstate_y, kstate_z, tilt_Array);
		
		osMutexWait(Tilt_Mutex, osWaitForever);
		pitch = tilt_Array[0];
		roll = tilt_Array[1];
		osMutexRelease(Tilt_Mutex);
		
	}
	
}
/*----------------------------------------------------------------------------
 *      Accelerometer functions
 *---------------------------------------------------------------------------*/
	
/* --- Configuration functions --------------------------------------------- */
	
/**
   * @brief  A function used to configure the accelerometer
   * @param  LIS3DSHStruct: LIS3DSHS configuration struct
   * @param  LIS3DSH_InterruptConfigStruct: LIS3DSHS interrupt channel configuration struct
   * @retval None
   */
void Configure_LIS3DSH(LIS3DSH_InitTypeDef LIS3DSHStruct, LIS3DSH_DRYInterruptConfigTypeDef LIS3DSH_InterruptConfigStruct) {
	
	LIS3DSHStruct.Power_Mode_Output_DataRate = LIS3DSH_DATARATE_25;   /* Power down or active mode with output data rate 3.125 / 6.25 / 12.5 / 25 / 50 / 100 / 400 / 800 / 1600 HZ */
    LIS3DSHStruct.Axes_Enable = LIS3DSH_XYZ_ENABLE;                     /* Axes enable */
    LIS3DSHStruct.Continous_Update = LIS3DSH_ContinousUpdate_Disabled;  /* Block or update Low/High registers of data until all data is read */
	LIS3DSHStruct.AA_Filter_BW = LIS3DSH_AA_BW_50;			            /* Choose anti-aliasing filter BW 800 / 400 / 200 / 50 Hz*/
    LIS3DSHStruct.Full_Scale = LIS3DSH_FULLSCALE_2;                     /* Full scale 2 / 4 / 6 / 8 / 16 g */
    LIS3DSHStruct.Self_Test = LIS3DSH_SELFTEST_NORMAL;                  /* Self test */
	
	LIS3DSH_InterruptConfigStruct.Dataready_Interrupt = LIS3DSH_DATA_READY_INTERRUPT_ENABLED;  /* Enable/Disable data ready interrupt */
    LIS3DSH_InterruptConfigStruct.Interrupt_signal = LIS3DSH_ACTIVE_HIGH_INTERRUPT_SIGNAL;     /* Interrupt Signal Active Low / Active High */
    LIS3DSH_InterruptConfigStruct.Interrupt_type = LIS3DSH_INTERRUPT_REQUEST_PULSED;           /* Interrupt type as latched or pulsed */
	
	LIS3DSH_Init(&LIS3DSHStruct);
	LIS3DSH_DataReadyInterruptConfig(&LIS3DSH_InterruptConfigStruct);
	
}

/**
   * @brief  A function used to configure the interrupt line of the accelerometer
   * @param  GPIO_E_Init: GPIOE configuration struct
   * @retval None
   */
void Configure_LIS3DSH_Interrupt_Line(GPIO_InitTypeDef GPIO_E_Init_Accelerometer) {
	
	__HAL_RCC_GPIOE_CLK_ENABLE();
	
	GPIO_E_Init_Accelerometer.Pin = GPIO_PIN_0;                    /* Specifies the GPIO pins to be configured */
    GPIO_E_Init_Accelerometer.Mode = GPIO_MODE_IT_RISING;          /* Specifies the operating mode for the selected pins */
    GPIO_E_Init_Accelerometer.Pull = GPIO_PULLDOWN;                /* Specifies the Pull-up or Pull-Down activation for the selected pins */
    GPIO_E_Init_Accelerometer.Speed = GPIO_SPEED_FREQ_LOW;         /* Specifies the speed for the selected pins */
                            
	HAL_GPIO_Init(GPIOE, &GPIO_E_Init_Accelerometer);
	
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 1);                        /* Priority of level 1 for the accelerometer */
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
	
}


/* --- Total initialization function --------------------------------------- */

/**
   * @brief  A function used to configure and initialize the display GPIO pins, the TIM3 timer and their flags
   * @param  None
   * @retval None
   */	
void Initialize_Accelerometer(void) {
	
	/* Configure the accelerometer */
	Configure_LIS3DSH(LIS3DSHStruct, LIS3DSH_InterruptConfigStruct);
	Configure_LIS3DSH_Interrupt_Line(GPIO_E_Init_Accelerometer);
	
	/* Initialize the Kalman filters (x, y and z)*/
	kstate_x = __init__(0.01, 1.5, 0.3, 0, 0, kstate_x);
	kstate_y = __init__(0.01, 1.5, 0.3, 0, 0, kstate_y);
	kstate_z = __init__(0.01, 1.5, 0.3, 0, 0, kstate_z);
	
	/* Initialize double tap variables */
	Reset_Tap = 0;
	Tap_Counter = 0;
	Detected_Double_Tap_Flag = 0;
	
	/* Initialize double tap buffer */
	acc_buf = malloc(buffer_size * sizeof(int));
	for(i = 0; i< buffer_size; i++) {
		acc_buf[i] = 1000;
	}
	
}


/* --- Accelerometer functions --------------------------------------------- */

/**
   * @brief  A function used to calculate the tilt
   * @param  kstate_x: Pointer to the Kalman filter of the x acceleration
   * @param  kstate_y: Pointer to the Kalman filter of the y acceleration
   * @param  kstate_z: Pointer to the Kalman filter of the z acceleration
   * @param  tilt_Array: Array used to store the pitch and roll values
   * @retval Measured tilt
   */
void Calculate_Tilt(kalman_state* kstate_x, kalman_state* kstate_y, kalman_state* kstate_z, double tilt_Array[2]) {
	
	float accelerations[3];
	float Ax, Ay, Az;
	double Ax1, Ay1, Az1;
	double pitch, roll;
	double Az_tap;

	/* Read the acceleration in the x, y and z directions */
	LIS3DSH_ReadACC(accelerations);
	
	/* Raw values*/
	Ax = accelerations[0];
	Ay = accelerations[1];
	Az = accelerations[2];
	
	/* Calibrated values*/
	Ax1 = (double) Ax * (0.001001487971751) + (double) Ay * (-0.000026697002793) + (double) Az * (0.000004161225213) - 0.014532292617973;
	Ay1 = (double) Ax * (-0.000012886855747) + (double) Ay * (0.000971123491151) + (double) Az * (0.000002812607406) + 0.000585555300052;
	Az1 = (double) Ax * (0.000007862071445) + (double) Ay * (-0.000004490648009) + (double) Az * (0.000960328656662) - 0.017547993185863;
	
	Az_tap = Az1*1000;
	
	/* Filtered values */
	Ax1 = Kalmanfilter_C(Ax1, kstate_x);
	Ay1 = Kalmanfilter_C(Ay1, kstate_y);	
	//Az1 = Kalmanfilter_C(Az1, kstate_z);
		
	/* Calculate the pitch */
	pitch = atan(Ax1 / sqrt((Ay1 * Ay1) + (Az1 * Az1))) * (180 / M_PI);
	
	/* Calculate the roll */
	roll = atan(Ay1 / sqrt((Ax1 * Ax1) + (Az1 * Az1))) * (180 / M_PI);
	
	/* 
	   | Sign of Z     | Sign of X | Quadrant |
	   |		+      |     -     |   1st    |
	   |		-      |     -     |   2nd    |
	   |		-      |     +     |   3rd    |
	   |		+      |     +     |   4th    | 
	*/
	
	if((accelerations[2] >= 0) && (accelerations[0] <= 0)) {
		tilt_Array[0] = - pitch;      // First quadrant (0 to 90)
	}
	else if ((accelerations[2] <= 0) && (accelerations[0] <= 0)) {
		tilt_Array[0] = 180 + pitch;  // Second quadrant (90 to 180)
	}
	else if ((accelerations[2] <= 0) && (accelerations[0] >= 0)) {
		tilt_Array[0] = 180 + pitch;  // Third quadrant (180 to 270)
	}
	else {
		tilt_Array[0] = 360 - pitch;  // Fourth quadrant (270 to 360)
	}
	
	/* 
	   | Sign of Z     | Sign of Y | Quadrant |
	   |		+      |     -     |   1st    |
	   |		-      |     -     |   2nd    |
	   |		-      |     +     |   3rd    |
	   |		+      |     +     |   4th    | 
	*/

	if((accelerations[2] >= 0) && (accelerations[1] <= 0)) {
		tilt_Array[1] = - roll;       // First quadrant (0 to 90)
	}
	else if ((accelerations[2] <= 0) && (accelerations[1] <= 0)) {
		tilt_Array[1] = 180 + roll;   // Second quadrant (90 to 180)
	}
	else if ((accelerations[2] <= 0) && (accelerations[1] >= 0)) {
		tilt_Array[1] = 180 + roll;   // Third quadrant (180 to 270)
	}
	else {
		tilt_Array[1] = 360 - roll;   // Fourth quadrant (270 to 360)
	}
	
	Detect_Double_Tap(Az_tap, pitch, roll);
	
}

/**
   * @brief  A function that initializes the properties of a kalman_state struct and returns a pointer to it
   * @param  q: Process noise covariance
   * @param  r: Measurement noise covariance
   * @param  p: Estimation error covariance
   * @param  k: Kalman gain
   * @param  x: Measurements
   * @retval Pointer to kalman_state struct
   */
kalman_state *__init__ (double q, double r, double p, double k, double initial_value, kalman_state* kstate) {
	
	kstate = malloc(sizeof(kalman_state));
	kstate->q = q;
	kstate->r = r;
	kstate->p = p;
	kstate->k = k;
	kstate->x = initial_value;
	return(kstate);
	
}

/**
   * @brief  Updates the properties of a variable of type kalman_state and returns a filtered value
   * @param  measurement: Temperature obtained from the temperature sensor through the ADC 
   * @param  kstate: Kalman filter state struct
   * @retval Filtered value
   */
double Kalmanfilter_C(double measurement, kalman_state* kstate) {
	
	kstate->p = kstate->p + kstate->q;
	kstate->k = kstate->p / (kstate->p + kstate->r);
	kstate->x = kstate->x + (kstate->k * (measurement - kstate->x));
	kstate->p = (1 - kstate->k) * kstate->p;
	return(kstate->x); 
	
}

/**
   * @brief  Detects if a double tap occurred and sets the double tap pin high to let the master know
   * @param  Z_Acceleration: Most recent measurement of the acceleration in the z direction
   * @param  pitch: Most recent pitch measurement
   * @param  roll: Most recent roll measurement
   * @retval NONE
   */
void Detect_Double_Tap(double Z_Acceleration, double pitch, double roll) {
	double Z_temp;
	Z_temp = Z_Acceleration;
	
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET);
	
	if (abs(pitch) < 10 && abs(roll) < 10 ){
		
		if(!tap_detected) {
			
			if(abs(Z_temp - avg) > 100) {
				
				if (incr_counter(Reset_Tap, 20) == buf_counter && Tap_Counter == 1){
					Tap_Counter = 0;
				}
				
				tap_detected = 1;
				tap_latch = buf_counter;
				
				Tap_Counter++;
		
				if(Tap_Counter == 1){
					Reset_Tap = buf_counter;
				}
				else if (Tap_Counter == 2){
					Detected_Double_Tap_Flag++;
					HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET);
					Tap_Counter = 0;
				}
			}	
		}
	}
	else {
		if(incr_counter(tap_latch, 3) == buf_counter) {
			Z_temp = avg;
		}
		else {
			tap_detected = 0;
		}
	}
	buf_counter = incr_counter(buf_counter, 1);
	acc_buf[buf_counter] = Z_temp;  //this is a circular buffer storing the past [buffer_size] z-accelerations
	
	sum = 0;
	for(i = 0; i<buffer_size; i++) {
		sum += acc_buf[i];		
	}
	avg = sum / buffer_size;
	
}

/**
   * @brief  Increment or decrement the modulo counter for the circular buffer
   * @param  src : counter to be changed
   * @param  amt : amt to be incremented, positive or negative. Should not be larger in magnitude than the buffer size.
   * @retval Incremented counter
   */
int incr_counter(int src, int amt) {
	int tmp;
	tmp = src + amt;
	if(tmp >= 0 && tmp < buffer_size) {
		return tmp;
	} else if(tmp >= buffer_size) {
		return tmp - buffer_size;
	} else if(tmp < 0) {
		return tmp + buffer_size;
	}
	return -1;
}

/* --- Handlers ------------------------------------------------------------ */

/**
  * @brief  This function handles EXTI0 interrupt requests.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void) {
	
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);

}

/* --- Callbacks ----------------------------------------------------------- */

/**
   * @brief  A function used to handle interrupts from the master and the accelerometer
   * @param  GPIO_Pin: GPIO pin that triggered the interrupt
   * @retval None
   */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	
	// Accelerometer has new values
	if (GPIO_Pin == GPIO_PIN_0) {
		// Signal that x, y and z acceleration values can be read
		osSignalSet(tid_Thread_Accelerometer, 0x01);
	}
	
	// Master wants something
	else if (GPIO_Pin == GPIO_PIN_2) {
		osSignalSet(tid_Thread_Communication, 0x01);
	}
	
}

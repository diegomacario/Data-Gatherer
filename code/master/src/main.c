#include "cube_hal.h"

#include "osal.h"
#include "sensor_service.h"
#include "debug.h"
#include "stm32_bluenrg_ble.h"
#include "bluenrg_utils.h"

#include <string.h>
#include <stdio.h>

#define TEMPERATURE_INSTRUCTION 31
#define PITCH_INSTRUCTION 30
#define ROLL_INSTRUCTION 29

/* ***************************** SPI3 Variables and Prototypes ***************************** */

/* --- SPI2 Struct --- */
SPI_HandleTypeDef SPI2_Handle;

/* ---  Functions  --- */

/* These functions are used to configure SPI3 and it associated pins */
void SPI3_Init(void);
void Configure_I_Want_Something_Pin(void);
void Configure_Slave_Is_Ready_Pin(void);
void Configure_Control_Pin(void);
void Configure_Double_Tap_Pin(void);
void sendInstruction(int instruct);

/* These functions are used to obtain bytes and covert them to floats */
void Get_Value(void);

/* -----  Flags  ----- */

int Slave_Sent_Data_Flag;
int Transfer_In_Progress_Flag;
int instruction;

uint8_t counter;

/* ----- Global variables ----- */

extern float global_pitch;
extern float global_roll;
extern float global_temperature;

extern int LED_Flag;
extern int Double_Tap_Flag;

/* ***************************************************************************************** */

/* Private defines -----------------------------------------------------------*/
#define BDADDR_SIZE 6

/* Private variables ---------------------------------------------------------*/
extern volatile uint8_t set_connectable;
extern volatile int connected;
uint8_t bnrg_expansion_board = IDB04A1; /* At startup, suppose the X-NUCLEO-IDB04A1 is used */

/* Private function prototypes -----------------------------------------------*/
void User_Process(uint8_t data);

int main(void)
{
  const char *name = "BlueNRG";
  uint8_t SERVER_BDADDR[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x43};
  uint8_t bdaddr[BDADDR_SIZE];
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  
  uint8_t  hwVersion;
  uint16_t fwVersion;
  
  int ret;  
  
  /* STM32Cube HAL library initialization:
   *  - Configure the Flash prefetch, Flash preread and Buffer caches
   *  - Systick timer is configured by default as source of time base, but user 
   *    can eventually implement his proper time base source (a general purpose 
   *    timer for example or other time source), keeping in mind that Time base 
   *    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
   *    handled in milliseconds basis.
   *  - Low Level Initialization
   */
  HAL_Init();
  
  /* Configure the User Button in GPIO Mode */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_GPIO);
  
  /* Configure the system clock */
  /* SYSTEM CLOCK = 32 MHz */
  SystemClock_Config();

  /* Initialize the BlueNRG SPI driver */
  BNRG_SPI_Init();
  
  /* Initialize the BlueNRG HCI */
  HCI_Init();

  /* Reset BlueNRG hardware */
  BlueNRG_RST();
    
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);

  /* 
   * Reset BlueNRG again otherwise we won't
   * be able to change its MAC address.
   * aci_hal_write_config_data() must be the first
   * command after reset otherwise it will fail.
   */
  BlueNRG_RST();
  
  PRINTF("HWver %d, FWver %d", hwVersion, fwVersion);
	PRINTF("\n\n");
  
  if (hwVersion > 0x30) { /* X-NUCLEO-IDB05A1 expansion board is used */
    bnrg_expansion_board = IDB05A1; 
    /*
     * Change the MAC address to avoid issues with Android cache:
     * if different boards have the same MAC address, Android
     * applications unless you restart Bluetooth on tablet/phone
     */
    SERVER_BDADDR[5] = 0x02;
  }

  /* The Nucleo board must be configured as SERVER */
  Osal_MemCpy(bdaddr, SERVER_BDADDR, sizeof(SERVER_BDADDR));
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret){
    PRINTF("Setting BD_ADDR failed.\n");
  }
  
  ret = aci_gatt_init();    
  if(ret){
    PRINTF("GATT_Init failed.\n");
  }

  if (bnrg_expansion_board == IDB05A1) {
    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x03, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }
  else {
    ret = aci_gap_init_IDB04A1(GAP_PERIPHERAL_ROLE_IDB04A1, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  }

  if(ret != BLE_STATUS_SUCCESS){
    PRINTF("GAP_Init failed.\n");
  }

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   strlen(name), (uint8_t *)name);

  if(ret){
    PRINTF("aci_gatt_update_char_value failed.\n");            
    while(1);
  }
  
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL,
                                     7,
                                     16,
                                     USE_FIXED_PIN_FOR_PAIRING,
                                     123456,
                                     BONDING);
  if (ret == BLE_STATUS_SUCCESS) {
    PRINTF("BLE Stack Initialized.\n");
  }
  
  PRINTF("SERVER: BLE Stack Initialized\n");
  
  ret = Add_Acc_Service();
  
  if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Acc service added successfully.\n");
  else
    PRINTF("Error while adding Acc service.\n");
	
	ret = Add_Temperature_Service();
	
	if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Temperature service added successfully.\n");
  else
    PRINTF("Error while adding temperature service.\n");
	
	ret = Add_Double_Tap_Service();
	
	if(ret == BLE_STATUS_SUCCESS)
    PRINTF("Double tap service added successfully.\n");
  else
    PRINTF("Error while adding double tap service.\n");
	
	ret = Add_LED_Service();
	
	if(ret == BLE_STATUS_SUCCESS)
    PRINTF("LED service added successfully.\n");
  else
    PRINTF("Error while adding LED service.\n");

  /* Set output power level */
  ret = aci_hal_set_tx_power_level(1,4);

	/* ********************************** SPI3 Initialization ********************************** */
	
	/* Configure communication pins */
	Configure_I_Want_Something_Pin();
	Configure_Slave_Is_Ready_Pin();
	Configure_Control_Pin();
	Configure_Double_Tap_Pin();
	
	/* Configure SPI2 */
	SPI3_Init();

	Transfer_In_Progress_Flag = 0;
	counter = 0;
	
/* ***************************************************************************************** */
	
	LED_Flag = 77;
	
	while(1) {
		
    HCI_Process();
		if(set_connectable){
			setConnectable();
			set_connectable = FALSE;
		}
		
		sendInstruction(PITCH_INSTRUCTION);
		Get_Value();
		Pitch_Update(global_pitch);
		
		HAL_Delay(100);
		
		sendInstruction(ROLL_INSTRUCTION);
		Get_Value();
		Roll_Update(global_roll);
		
		HAL_Delay(100);
		
		sendInstruction(TEMPERATURE_INSTRUCTION);
		Get_Value();
		Temperature_Update(global_temperature);
		
		HAL_Delay(100);
		
		if(LED_Flag != 77) {
			printf("%d\n", LED_Flag);
			sendInstruction(LED_Flag);
			Get_Value();
			Temperature_Update(global_temperature);
			LED_Flag = 77;
		}
		
		if(Double_Tap_Flag == 1) {
		
			Double_Tap_Notify();
			printf("Notified!");
			Double_Tap_Flag = 0;
			
		}
		
  }
}


/* ************************************* SPI3 Functions ************************************ */

/**
  * @brief  This function configures the SPI2
  * @param  None
  * @retval None
  */
void SPI3_Init(void) {

  __HAL_RCC_SPI3_CLK_ENABLE();
	
  HAL_SPI_DeInit(&SPI2_Handle);
  SPI2_Handle.Instance 		            = SPI3;
	
  SPI2_Handle.Init.BaudRatePrescaler 	= SPI_BAUDRATEPRESCALER_256;
	
  SPI2_Handle.Init.Direction 			= SPI_DIRECTION_2LINES;
  SPI2_Handle.Init.CLKPhase 			= SPI_PHASE_1EDGE;
  SPI2_Handle.Init.CLKPolarity 			= SPI_POLARITY_LOW;
  SPI2_Handle.Init.CRCCalculation		= SPI_CRCCALCULATION_DISABLED;
  SPI2_Handle.Init.CRCPolynomial 		= 7;
  SPI2_Handle.Init.DataSize 			= SPI_DATASIZE_8BIT;
  SPI2_Handle.Init.FirstBit 			= SPI_FIRSTBIT_MSB;
	
  SPI2_Handle.Init.NSS 					= SPI_NSS_SOFT;
	
  SPI2_Handle.Init.TIMode 				= SPI_TIMODE_DISABLED;
  SPI2_Handle.Init.Mode 				= SPI_MODE_MASTER;
	
  if (HAL_SPI_Init(&SPI2_Handle) != HAL_OK) {printf ("ERROR: Error in initialising SPI2 \n");};
  
  __HAL_SPI_ENABLE(&SPI2_Handle);
  
}

/**
  * @brief  Configure the pin used by the master to pass information to the slave
  * @param  None
  * @retval None
  */
void Configure_Control_Pin(void) {
	
	GPIO_InitTypeDef GPIO_Control_Pin;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_Control_Pin.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_6 | GPIO_PIN_8;  
    GPIO_Control_Pin.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Control_Pin.Pull = GPIO_NOPULL;
    GPIO_Control_Pin.Speed = GPIO_SPEED_FREQ_MEDIUM;

	HAL_GPIO_Init(GPIOC, &GPIO_Control_Pin);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET);
}

/**
  * @brief  Configure the pin used by the master to tell the  it wants something
  * @param  None
  * @retval None
  */
void Configure_I_Want_Something_Pin(void) {
	
	GPIO_InitTypeDef GPIO_I_Want_This_Pin;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	/* Pin C0 is used by the master to tell the slave it wants something */
	GPIO_I_Want_This_Pin.Pin = GPIO_PIN_0;  
    GPIO_I_Want_This_Pin.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_I_Want_This_Pin.Pull = GPIO_NOPULL;
    GPIO_I_Want_This_Pin.Speed = GPIO_SPEED_FREQ_MEDIUM;

	HAL_GPIO_Init(GPIOC, &GPIO_I_Want_This_Pin);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	
}

/**
  * @brief  Configure the pin used by the  to tell me master it is ready to send it what it wants
  * @param  None
  * @retval None
  */
void Configure_Slave_Is_Ready_Pin(void) {
	
	/* Pin C1 is used by the master to know when the slave is ready to send it what it wants */
	
	GPIO_InitTypeDef GPIO_I_Got_It_Pin;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_I_Got_It_Pin.Pin = GPIO_PIN_1; 
    GPIO_I_Got_It_Pin.Mode = GPIO_MODE_IT_RISING;
    GPIO_I_Got_It_Pin.Pull = GPIO_PULLDOWN;
    GPIO_I_Got_It_Pin.Speed = GPIO_SPEED_FREQ_MEDIUM;
	
	HAL_GPIO_Init(GPIOC, &GPIO_I_Got_It_Pin);
	
	/* Priority of level 1 for C1 */
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	
}

void Configure_Double_Tap_Pin() {

	/* Pin C1 is used by the master to know when the slave is ready to send it what it wants */
	
	GPIO_InitTypeDef GPIO_Double_Tap_Pin;
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	
	GPIO_Double_Tap_Pin.Pin = GPIO_PIN_7; 
    GPIO_Double_Tap_Pin.Mode = GPIO_MODE_IT_RISING;
    GPIO_Double_Tap_Pin.Pull = GPIO_PULLDOWN;
    GPIO_Double_Tap_Pin.Speed = GPIO_SPEED_FREQ_MEDIUM;
	
	HAL_GPIO_Init(GPIOC, &GPIO_Double_Tap_Pin);
	
	/* Priority of level 1 for C1 */
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

void Get_Value(void) {

	Transfer_In_Progress_Flag = 1;
	Slave_Sent_Data_Flag = 0;
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET); // Signal that you are going to ask for something
		
}

void sendInstruction(int instruct){ // 0-10 for numbers 0 to 10, 11-20 for -1 to -10, 31 for temperature, 30 for pitch, 29 for roll
	if(instruct > 31){
		printf("ERROR Invalid instruction\n");}
	else{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_RESET);
		switch(instruct){
			case 31: // Temperature
				instruction = TEMPERATURE_INSTRUCTION;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 30: // Pitch
				instruction = PITCH_INSTRUCTION;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_6, GPIO_PIN_SET);
				break;
			case 29: // Roll
				instruction = ROLL_INSTRUCTION;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 0:
				break;
			case 1:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 2:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
				break;
			case 3:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 4:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
				break;
			case 5:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 6:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_6, GPIO_PIN_SET);
				break;
			case 7:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 8:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
				break;
			case 9:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 10:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_6 , GPIO_PIN_SET);
				break;
			case 11:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 12:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_13, GPIO_PIN_SET);
				break;
			case 13:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 14:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_6, GPIO_PIN_SET);
				break;
			case 15:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5 | GPIO_PIN_13 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 16:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
				break;
			case 17:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 18:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_6, GPIO_PIN_SET);
				break;
			case 19:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 20:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_13, GPIO_PIN_SET);
				break;
			case 21:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_13 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 22:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_13 | GPIO_PIN_6, GPIO_PIN_SET);
				break;
			case 23:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_13 | GPIO_PIN_6 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			case 24:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
				break;
			case 25:
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8, GPIO_PIN_SET);
				break;
			default:
				break;
		}
	}
}

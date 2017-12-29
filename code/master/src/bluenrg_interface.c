#include "bluenrg_interface.h"

#include "debug.h"
#include "ble_status.h"
#include "hci.h"
#include "stm32_bluenrg_ble.h"
#include "sensor_service.h"

#define TEMPERATURE_INSTRUCTION 31
#define PITCH_INSTRUCTION 30
#define ROLL_INSTRUCTION 29

extern SPI_HandleTypeDef SpiHandle;
extern SPI_HandleTypeDef SPI2_Handle;

/* Prototypes */
void Receive_Data(void);
extern tBleStatus Double_Tap_Notify(void);

/* Variables and flags */
extern int Transfer_In_Progress_Flag;
extern int Slave_Sent_Data_Flag;

float global_pitch;
float global_roll;
float global_temperature;

int Double_Tap_Flag = 0;

uint8_t result;
uint8_t slave_data[4];
int byte_counter = 0;
int first_time = 1;
int dummy_ctr = 0;

/**
 * @brief  EXTI line detection callback.
 * @param  Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_1 && Transfer_In_Progress_Flag == 1 && Slave_Sent_Data_Flag == 0) {
	
		Receive_Data();
		Slave_Sent_Data_Flag = 1;
		Transfer_In_Progress_Flag = 0;
		
	}
	if (GPIO_Pin == GPIO_PIN_7) {
		
		printf("Double tap!\n");
		Double_Tap_Flag = 1;
		
	}
	else { 
		
		HCI_Isr();
	
	}
}

extern int instruction;
void Receive_Data(void) {
	
	byte_counter = 0;
	float recv_val; 
		
	result = SPI2_Handle.Instance->DR;
	while (byte_counter < 4) {
		
		while (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_RXNE) == RESET) {
			if (__HAL_SPI_GET_FLAG(&SPI2_Handle, SPI_FLAG_TXE) == 1) {
				SPI2_Handle.Instance->DR = 0;
			}
		}
		
		slave_data[byte_counter] = SPI2_Handle.Instance->DR;
		byte_counter++;
		
	}
	
	union {
		float f;
		uint8_t bytes[4];
	} u;
	
	u.bytes[0] = slave_data[0]; 
	u.bytes[1] = slave_data[1]; 
	u.bytes[2] = slave_data[2]; 
	u.bytes[3] = slave_data[3];
	recv_val = u.f;
	
	if(instruction == TEMPERATURE_INSTRUCTION){
		global_temperature = recv_val;
		//printf("Temperature: ");
	}
	else if(instruction == PITCH_INSTRUCTION){
		global_pitch = recv_val; 
		//printf("Pitch: ");
	}
	else if(instruction == ROLL_INSTRUCTION){
		global_roll = recv_val;
		//printf("Roll: ");
	}
	
	//printf("%f\n", recv_val);
	
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET); // Reset the "I want something" signal
}

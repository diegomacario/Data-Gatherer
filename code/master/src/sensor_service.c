#include "sensor_service.h"

/* Private variables ---------------------------------------------------------*/
volatile int connected = FALSE;
volatile uint8_t set_connectable = 1;
volatile uint16_t connection_handle = 0;
volatile uint8_t notification_enabled = FALSE;
volatile AxesRaw_t axes_data = {0, 0, 0};
extern uint8_t bnrg_expansion_board;
uint16_t sampleServHandle, TXCharHandle, RXCharHandle;
uint16_t accServHandle, rollCharHandle, pitchCharHandle;
uint16_t temperatureServHandle, temperatureCharHandle;
uint16_t doubleTapServHandle, doubleTapCharHandle;
uint16_t ledServHandle, patternCharHandle, intensityCharHandle;

extern void sendInstruction(int instruct);
extern void Get_Value(void);
int LED_Flag;

/* Private macros ------------------------------------------------------------*/
#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0]  = uuid_0;  uuid_struct[1]  = uuid_1;  uuid_struct[2]  = uuid_2;  uuid_struct[3]  = uuid_3;  \
    uuid_struct[4]  = uuid_4;  uuid_struct[5]  = uuid_5;  uuid_struct[6]  = uuid_6;  uuid_struct[7]  = uuid_7;  \
    uuid_struct[8]  = uuid_8;  uuid_struct[9]  = uuid_9;  uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)


  #define COPY_ACC_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x02,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_ROLL_UUID(uuid_struct)   			COPY_UUID_128(uuid_struct,0xe2,0x3e,0x78,0xa0, 0xcf,0x4a, 0x11,0xe1, 0x8f,0xfc, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_PITCH_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x34,0x0a,0x1b,0x80, 0xcf,0x4b, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	
	#define COPY_TEMPERATURE_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x05,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_TEMPERATURE_CHAR_UUID(uuid_struct)  		COPY_UUID_128(uuid_struct,0x05,0x36,0x00,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	
	#define COPY_DOUBLE_TAP_SERVICE_UUID(uuid_struct)  	COPY_UUID_128(uuid_struct,0xF5,0x36,0x6e,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	#define COPY_DOUBLE_TAP_CHAR_UUID(uuid_struct)  		COPY_UUID_128(uuid_struct,0xF5,0x36,0x00,0x80, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
	
	#define COPY_LED_SERVICE_UUID(uuid_struct)  	COPY_UUID_128(uuid_struct,0x02,0x36,0x6e,0x01, 0xcf,0x3a, 0x11,0xe1, 0x9a,0xb4, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_PATTERN_UUID(uuid_struct)   			COPY_UUID_128(uuid_struct,0xe2,0x3e,0x78,0x42, 0xcf,0x4a, 0x11,0xe1, 0x8f,0xfc, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)
  #define COPY_INTENSITY_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x34,0x0a,0x1b,0x56, 0xcf,0x4b, 0x11,0xe1, 0xac,0x36, 0x00,0x02,0xa5,0xd5,0xc5,0x1b)


/* Store Value into a buffer in Little Endian Format */
#define STORE_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/**
 * @brief  Add an accelerometer service using a vendor specific profile.
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_Acc_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_ACC_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &accServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  COPY_ROLL_UUID(uuid);
  ret =  aci_gatt_add_char(accServHandle, UUID_TYPE_128, uuid, 4,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ, 
													 ATTR_PERMISSION_NONE,
													 0,
                           16, 0, &rollCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  COPY_PITCH_UUID(uuid);  
  ret =  aci_gatt_add_char(accServHandle, UUID_TYPE_128, uuid, 4,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           0,
                           16, 0, &pitchCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  
  PRINTF("Service ACC added. Handle 0x%04X, Free fall Charac handle: 0x%04X, Acc Charac handle: 0x%04X\n",accServHandle, rollCharHandle, pitchCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding ACC service.\n");
  return BLE_STATUS_ERROR ;
    
}

/**
 * @brief  Update the roll characteristic value.
 * @param  angle data in unsigned char
 * @retval tBleStatus Status
 */
tBleStatus Roll_Update(float data)
{  
  tBleStatus ret;    
	union{
		float a;
    unsigned char bytes[4];
	}roll;
	
	roll.a = data;
	unsigned char correctlyOrderedBytes[4] = { roll.bytes[3], roll.bytes[2], roll.bytes[1], roll.bytes[0]};
  ret = aci_gatt_update_char_value(accServHandle, rollCharHandle, 0, 4, correctlyOrderedBytes);
	
  if (ret != BLE_STATUS_SUCCESS){
    //PRINTF("Error while updating roll characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Update the pitch characteristic value.
 * @param  angle data in unsigned char
 * @retval Status
 */
tBleStatus Pitch_Update(float data)
{  
  tBleStatus ret;    
	
	union{
		float a;
    unsigned char bytes[4];
	}pitch;
	
	pitch.a = data;
	unsigned char correctlyOrderedBytes[4] = { pitch.bytes[3], pitch.bytes[2], pitch.bytes[1], pitch.bytes[0]};
  ret = aci_gatt_update_char_value(accServHandle, pitchCharHandle, 0, 4, correctlyOrderedBytes);
	
  if (ret != BLE_STATUS_SUCCESS){
    //PRINTF("Error while updating pitch characteristic.\n");
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Add a temperature service.
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_Temperature_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_TEMPERATURE_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &temperatureServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  COPY_TEMPERATURE_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(temperatureServHandle, UUID_TYPE_128, uuid, 4,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ, 
													 ATTR_PERMISSION_NONE,
													 0,
                           16, 0, &temperatureCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  return BLE_STATUS_SUCCESS;
	
fail:
  PRINTF("Error while adding ACC service.\n");
  return BLE_STATUS_ERROR ;
    
}

/**
 * @brief  Update the temperature characteristic value.
 * @param  angle data in unsigned char
 * @retval tBleStatus Status
 */
tBleStatus Temperature_Update(float data)
{  
  tBleStatus ret;    
	union{
		float a;
    unsigned char bytes[4];
	}temp;
	
	temp.a = data;
	unsigned char correctlyOrderedBytes[4] = { temp.bytes[3], temp.bytes[2], temp.bytes[1], temp.bytes[0]};
  ret = aci_gatt_update_char_value(temperatureServHandle, temperatureCharHandle, 0, 4, correctlyOrderedBytes);
	
  if (ret != BLE_STATUS_SUCCESS){
    //PRINTF("Error while updating temperature characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;	
}

/**
 * @brief  Add a double tap service.
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_Double_Tap_Service(void)
{
  tBleStatus ret;

  uint8_t uuid[16];
  
  COPY_DOUBLE_TAP_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7,
                          &doubleTapServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
  
  COPY_DOUBLE_TAP_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(doubleTapServHandle, UUID_TYPE_128, uuid, 1,
                           CHAR_PROP_NOTIFY, 
													 ATTR_PERMISSION_NONE,
													 0,
                           16, 0, &doubleTapCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;
  return BLE_STATUS_SUCCESS;
	
fail:
  PRINTF("Error while adding ACC service.\n");
  return BLE_STATUS_ERROR ;
    
}

/**
 * @brief  Send notify for double tap.
 * @param  void
 * @retval tBleStatus Status
 */
uint8_t val = 0x00;
tBleStatus Double_Tap_Notify(void)
{  
  tBleStatus ret;    
	val = (val == 0x00) ? 0x01 : 0x00;
	
  ret = aci_gatt_update_char_value(doubleTapServHandle, doubleTapCharHandle, 0, 1, &val);
	
  if (ret != BLE_STATUS_SUCCESS){
    //PRINTF("Error while updating roll characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;	
}

/*
 * @brief  Add LED service using a vendor specific profile.
 * @param  None
 * @retval Status
 */
tBleStatus Add_LED_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  
  COPY_LED_SERVICE_UUID(uuid);

  ret = aci_gatt_add_serv(UUID_TYPE_128, uuid, PRIMARY_SERVICE, 7,
                          &ledServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;    
   
  COPY_PATTERN_UUID(uuid);
  ret =  aci_gatt_add_char(ledServHandle, UUID_TYPE_128, uuid, 1,
                           CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &patternCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;  
	
	COPY_INTENSITY_UUID(uuid);
  ret =  aci_gatt_add_char(ledServHandle, UUID_TYPE_128, uuid, 1,
                           CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &intensityCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;  
  
  PRINTF("Service LED BUTTON added. Handle 0x%04X, LED button Charac handle: 0x%04X\n",ledServHandle, patternCharHandle);	
  return BLE_STATUS_SUCCESS; 
  
fail:
  PRINTF("Error while adding LED service.\n");
  return BLE_STATUS_ERROR;
}

/**
 * @brief  This function is called attribute value corresponding to 
 *         patternCharHandle or intensity characteristic gets modified.
 * @param  Handle of the attribute
 * @param  Size of the modified attribute data
 * @param  Pointer to the modified attribute data
 * @retval None
 */
void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if(handle == patternCharHandle +1){
		
		//printf("Pattern: %u\n", att_data[0]);
		
		if(att_data[0] == 0){ 		  // Off
			//sendInstruction(21);
			LED_Flag = 21;
		}
		else if(att_data[0] == 1){  // All on
			//sendInstruction(22);
			LED_Flag = 22;
		}
		else if(att_data[0] == 2){  // Rotating
			//sendInstruction(23);
			LED_Flag = 23;
		}
		else if(att_data[0] == 3){  // Custom
			//sendInstruction(24);
			LED_Flag = 24;
		}
		else if(att_data[0] == 4){  // Glow
			//sendInstruction(25);
			LED_Flag = 25;
		}
		else {
			//sendInstruction(21);
			LED_Flag = 21;
		}
		//Get_Value();
  }
	
	else if(handle == intensityCharHandle +1){
		
		//printf("Intensity: %u\n", att_data[0]);
		
		if(att_data[0] == 0){ 		  
			//sendInstruction(0);
			LED_Flag = 0;      
		}
		else if(att_data[0] == 1){  
			//sendInstruction(1);
			LED_Flag = 1;
		}
		else if(att_data[0] == 2){  
			//sendInstruction(2);
			LED_Flag = 2;
		}
		else if(att_data[0] == 3){  
			//sendInstruction(3);
			LED_Flag = 3;
		}
		else if(att_data[0] == 4){  
			//sendInstruction(4);
			LED_Flag = 4;
		}
		else if(att_data[0] == 5){ 
			//sendInstruction(5);
			LED_Flag = 5;
		}
		else if(att_data[0] == 6){ 
			//sendInstruction(6);
			LED_Flag = 6;
		}
		else if(att_data[0] == 7){  
			//sendInstruction(7);
			LED_Flag = 7;
		}
		else if(att_data[0] == 8){  
			//sendInstruction(8);
			LED_Flag = 8;
		}
		else if(att_data[0] == 9){ 
			//sendInstruction(9);
			LED_Flag = 9;
		}
		else if(att_data[0] == 10){  
			//sendInstruction(10);
			LED_Flag = 10;
		}
		else if(att_data[0] == 11){ // -1 
			//sendInstruction(11);
			LED_Flag = 11;
		}
		else if(att_data[0] == 12){ // -2 
			//sendInstruction(12);
			LED_Flag = 12;
		}
		else if(att_data[0] == 13){ // -3
			//sendInstruction(13);
			LED_Flag = 13;
		}
		else if(att_data[0] == 14){ // -4
			//sendInstruction(14);
			LED_Flag = 14;
		}
		else if(att_data[0] == 15){ // -5
			//sendInstruction(15);
			LED_Flag = 15;
		}
		else if(att_data[0] == 16){ // -6 
			//sendInstruction(16);
			LED_Flag = 16;
		}
		else if(att_data[0] == 17){ // -7 
			//sendInstruction(17);
			LED_Flag = 17;
		}
		else if(att_data[0] == 18){ // -8 
			//sendInstruction(18);
			LED_Flag = 18;
		}
		else if(att_data[0] == 19){ // -9 
			//sendInstruction(19);
			LED_Flag = 19;
		}
		else if(att_data[0] == 20){ // -10 
			//sendInstruction(20);
			LED_Flag = 20;
		}
		//Get_Value();
  }
}

/**
 * @brief  Puts the device in connectable mode.
 *         If you want to specify a UUID list in the advertising data, those data can
 *         be specified as a parameter in aci_gap_set_discoverable().
 *         For manufacture data, aci_gap_update_adv_data must be called.
 * @param  None 
 * @retval None
 */
void setConnectable(void)
{  
  tBleStatus ret;
  
  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'G','0','5'};
  
  /* disable scan response */
  hci_le_set_scan_resp_data(0,NULL);
  PRINTF("General Discoverable Mode.\n");
  
  ret = aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);
  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error while setting discoverable mode (%d)\n", ret);    
  }  
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  uint8_t Address of peer device
 * @param  uint16_t Connection handle
 * @retval None
 */
void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle)
{  
  connected = TRUE;
  connection_handle = handle;
  
  PRINTF("Connected to device:");
  for(int i = 5; i > 0; i--){
    PRINTF("%02X-", addr[i]);
  }
  PRINTF("%02X\n", addr[0]);
}

/**
 * @brief  This function is called when the peer device gets disconnected.
 * @param  None 
 * @retval None
 */
void GAP_DisconnectionComplete_CB(void)
{
  connected = FALSE;
  PRINTF("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
  notification_enabled = FALSE;
}

/**
 * @brief  Read request callback.
 * @param  uint16_t Handle of the attribute
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{  
  //EXIT:
  if(connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}

/**
 * @brief  Callback processing the ACI events.
 * @note   Inside this function each event must be identified and correctly
 *         parsed.
 * @param  void* Pointer to the ACI packet
 * @retval None
 */
void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = pckt;
  /* obtain event packet */
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
  
  if(hci_pckt->type != HCI_EVENT_PKT)
    return;
  
  switch(event_pckt->evt){
    
  case EVT_DISCONN_COMPLETE:
    {
      GAP_DisconnectionComplete_CB();
    }
    break;
    
  case EVT_LE_META_EVENT:
    {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      switch(evt->subevent){
      case EVT_LE_CONN_COMPLETE:
        {
          evt_le_connection_complete *cc = (void *)evt->data;
          GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
        }
        break;
      }
    }
    break;
    
  case EVT_VENDOR:
    {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;
      switch(blue_evt->ecode){
				
				case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:         
        {
          /* this callback is invoked when a GATT attribute is modified
          extract callback data and pass to suitable handler function */
          if (bnrg_expansion_board == IDB05A1) {
            evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
            Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data); 
          }
          else {
            evt_gatt_attr_modified_IDB04A1 *evt = (evt_gatt_attr_modified_IDB04A1*)blue_evt->data;
            Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data); 
          }                       
        }
        break; 

      case EVT_BLUE_GATT_READ_PERMIT_REQ:
        {
          evt_gatt_read_permit_req *pr = (void*)blue_evt->data;                    
          Read_Request_CB(pr->attr_handle);                    
        }
        break;
      }
    }
    break;
  }    
}

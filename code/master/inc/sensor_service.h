#ifndef _SENSOR_SERVICE_H_
#define _SENSOR_SERVICE_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"
#include "hal_types.h"
#include "bluenrg_gatt_server.h"
#include "bluenrg_gap.h"
#include "string.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gatt_aci.h"
#include "hci_const.h"
#include "gp_timer.h"
#include "bluenrg_hal_aci.h"
#include "bluenrg_aci_const.h"   
#include "hci.h"
#include "hal.h"
#include "sm.h"
#include "debug.h"

#include <stdlib.h>

/* Exported defines ----------------------------------------------------------*/   
#define IDB04A1 0
#define IDB05A1 1

/**
 * @brief Instantiate two new services:
 *        1. Timer Service with two characteristics
 *           - Seconds characteristic (Readable only)
 *           - Minutes characteristics (Readable and Notifiable)
 *        2. LED Button Service with one characteristic
 *           - LED characteristic (Readable and Writable)
 */
#define NEW_SERVICES 0

typedef int i32_t;

/** 
 * Structure containing acceleration value (in mg) of each axis.
 */
typedef struct {
  i32_t AXIS_X;
  i32_t AXIS_Y;
  i32_t AXIS_Z;
} AxesRaw_t;

tBleStatus Add_Acc_Service(void);
tBleStatus Roll_Update(float data);
tBleStatus Pitch_Update(float data);
tBleStatus Add_Temperature_Service(void);
tBleStatus Temperature_Update(float data);
tBleStatus Add_Double_Tap_Service(void);
tBleStatus Double_Tap_Notify(void);
tBleStatus Add_LED_Service(void);
void       setConnectable(void);
void       enableNotification(void);
void       GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle);
void       GAP_DisconnectionComplete_CB(void);
void       HCI_Event_CB(void *pckt);

#ifdef __cplusplus
}
#endif

#endif /* _SENSOR_SERVICE_H_ */

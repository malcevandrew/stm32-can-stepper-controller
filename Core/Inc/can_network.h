#ifndef CAN_NETWORK_H
#define CAN_NETWORK_H

#include "stm32f1xx_hal.h"

void CAN_Network_Init(void);
void CAN_Network_Update(void);
void CAN_Network_UpdateStatus(void);
void CAN_Network_SendExtendedStatus(uint8_t cmd_id, uint8_t tx_status, uint8_t state_code, int32_t position);

#endif // CAN_NETWORK_H
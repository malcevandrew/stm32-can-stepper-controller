#ifndef __CAN_NETWORK_H
#define __CAN_NETWORK_H

#include "main.h"

// 1. Функция для запуска CAN и настройки фильтра пакетов
void CAN_Network_Init(void);

// 2. Функция-диспетчер, которая будет проверять новые команды в главном цикле
void CAN_Network_Update(void);

void CAN_Network_SendStatus(uint8_t state_code, int32_t position);

void CAN_Network_UpdateStatus(void);

#endif /* __CAN_NETWORK_H */

#ifndef CAN_NETWORK_H
#define CAN_NETWORK_H

#include "stm32f1xx_hal.h"

// --- ПРОТОТИПЫ СИСТЕМНЫХ ФУНКЦИЙ ШИНЫ CAN ---

/**
 * @brief Инициализация аппаратного периферийного блока CAN и NVIC-фильтров кремния.
 * Настраивает узел №1 на прослушивание исключительно кадров с ID = 0x201.
 */
void CAN_Network_Init(void);

/**
 * @brief Фоновый диспетчер обработки входящих сетевых команд (HOME / GOTO / STOP).
 * Вызывается непрерывно в главном цикле while(1) файла main.c.
 */
void CAN_Network_Update(void);

/**
 * @brief Фоновый диспетчер периодической и событийной отправки статуса и телеметрии.
 * Вызывается непрерывно в главном цикле while(1) файла main.c.
 */
void CAN_Network_UpdateStatus(void);

/**
 * @brief Функция прямой транзакционной отправки расширенного 7-байтового кадра обратной связи.
 * Используется автоматом движений для отправки квитанций ACK (0xAA), NACK (0xFF) и DONE (0x55) Мастеру.
 * 
 * @param cmd_id     Эхо идентификатора транзакции (Token ID)
 * @param tx_status  Текущий статус транзакции (0xAA / 0xFF / 0x55)
 * @param state_code Код физического состояния конечного автомата привода (0x01..0x04)
 * @param position   Текущее физическое положение каретки в честных миллиметрах (мм)
 */
void CAN_Network_SendExtendedStatus(uint8_t cmd_id, uint8_t tx_status, uint8_t state_code, int32_t position);

#endif // CAN_NETWORK_H
#include "can_network.h"
#include "motion.h"
#include "main.h" 
#include <stdio.h>
#include <string.h>

// ============================================================================
#define CAN_NODE_ID                 0x01                // Наш адрес: Узел №1
#define RX_CAN_FILTER_ID        (0x200 + CAN_NODE_ID)   // Принимаем команды только с ID = 0x201
#define TX_CAN_STATUS_ID        (0x300 + CAN_NODE_ID)   // Отправляем ответы только с ID = 0x301
// ============================================================================

extern CAN_HandleTypeDef hcan;

static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8];                   // Буфер для хранения принятого кадра

// Переменные транзакции
volatile uint8_t can_new_command_flag = 0;  // Поступила новая команда
volatile uint8_t can_pending_command = 0;   // Значение принятой команды
volatile int32_t can_pending_target = 0;    // Значение принятой координаты

volatile uint8_t current_cmd_id = 0;        // ID текущего токена
volatile uint8_t last_processed_cmd_id = 0; // ID последнего завершенного токена
volatile uint8_t transaction_status = 0;    // Статус транзакции: 0xAA (ACK), 0xFF (NACK), 0x55 (DONE)

// Инициализация приема и запуск аппаратной фильтрации
void CAN_Network_Init(void) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; 
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 

    // Аппаратная фильтрация: ловим только ID = 0x201
    sFilterConfig.FilterIdHigh = (RX_CAN_FILTER_ID << 5);   // Принимаем команды только с ID = 0x201 
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = (0x7FF << 5);          // Проверка всех 11 бит на совпадение
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;      // Куда складывать пакеты (FIFO 0)
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    // Применение фильтра к периферии hcan
    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    // Запуск CAN-интерфейса
    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }

    // Активация прерываний на прием
    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    printf("[CAN] Node 0x01 (Stepper) active. Mask filter set to 0x201.\r\n");
}

// Отправка 7-байтового ответа (ID = 0x301)
void CAN_Network_SendExtendedStatus(uint8_t cmd_id, uint8_t tx_status, uint8_t state_code, int32_t position) {
    CAN_TxHeaderTypeDef local_tx_header;
    uint8_t local_tx_data[8];   // размер буфера 8 байт нужен для корректной работы HAL_CAN_AddTxMessage()
    uint32_t local_tx_mailbox;

    local_tx_header.StdId = TX_CAN_STATUS_ID; // Отвечаем на ID = 0x301
    local_tx_header.RTR = CAN_RTR_DATA;
    local_tx_header.IDE = CAN_ID_STD;
    local_tx_header.DLC = 7; 
    local_tx_header.TransmitGlobalTime = DISABLE;

    local_tx_data[0] = cmd_id;      // Токен транзакции
    local_tx_data[1] = tx_status;   // Состояние транзакции: ACK/NACK/DONE
    local_tx_data[2] = state_code;  // Физическое состояние автомата: 0x01..0x04

    // Реальная координата в мм (Big Endian)
    local_tx_data[3] = (uint8_t)((position >> 24) & 0xFF);
    local_tx_data[4] = (uint8_t)((position >> 16) & 0xFF);
    local_tx_data[5] = (uint8_t)((position >> 8)  & 0xFF);
    local_tx_data[6] = (uint8_t)(position         & 0xFF);

   
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
        HAL_CAN_AddTxMessage(&hcan, &local_tx_header, local_tx_data, &local_tx_mailbox);    // Передаем local_tx_data на отправку
    }
}

// Аппаратное прерывание приема кадра CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_interface) {

    // Записываем полученый кадр в буфер RxData
    if (HAL_CAN_GetRxMessage(hcan_interface, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        
        // Проверка что длина 6 байт
        if (!can_new_command_flag && RxHeader.DLC == 6) {
            uint8_t incoming_id = RxData[0];    // Токен
            uint8_t incoming_cmd = RxData[1];   // Команда

            // Cборка числа через беззнаковый буфер (Big Endian)
            uint32_t temp_target = ((uint32_t)RxData[2] << 24) |
                                   ((uint32_t)RxData[3] << 16) |
                                   ((uint32_t)RxData[4] << 8)  |
                                   ((uint32_t)RxData[5]);
            
            can_pending_target = (int32_t)temp_target;
            can_pending_command = incoming_cmd;
            current_cmd_id = incoming_id; 

            // Сбрасываем статус транзакции в прерывании
            // Как только пришел новый токен команды, мы принудительно переводим статус транзакции в состояние ожидания (0x00)
            transaction_status = 0x00;

            can_new_command_flag = 1; 
        }
    }
}

// Фоновый диспетчер циклической отправки статуса движения (каждые 50 мс)
void CAN_Network_UpdateStatus(void) {
    extern volatile ControllerState_t current_state;
    extern int32_t current_position;
    extern volatile uint8_t last_processed_cmd_id;
    extern volatile uint8_t current_cmd_id;
    extern volatile uint8_t transaction_status;

    static uint32_t last_can_send_tick = 0;
    static ControllerState_t last_state = STATE_STARTUP;
    
    uint32_t current_tick = HAL_GetTick(); 
    int32_t current_position_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);   // Переводим шаги в миллиметры с округлением

    // Шлем телеметрию каждые 50 мс для поддержания связи
    if (current_tick - last_can_send_tick >= 50) {
        last_can_send_tick = current_tick;

        uint8_t live_state_code = 0x04;                 // По умолчанию UNHOMED
        uint8_t live_tx_status = transaction_status;    // По умолчанию просто поток данных (ожидание)

        if (current_state == STATE_MOVING || current_state == STATE_HOMING) {
            live_state_code = 0x01;                     // MOVING
            live_tx_status = 0xAA;                      // Во время марша в эфир идет ACK
        } else {
            // Если стоим на месте, кодируем актуальное состояние
            switch (current_state) {
                case STATE_READY:   live_state_code = 0x02; live_tx_status = 0x00; break;   // Во время простоя шлем статус 0x00
                case STATE_FAULT:   live_state_code = 0x03; live_tx_status = 0xFF; break;
                case STATE_UNHOMED: live_state_code = 0x04; live_tx_status = 0x00; break;
                default:            live_state_code = 0x04; break;
            }
        }

        CAN_Network_SendExtendedStatus(current_cmd_id, live_tx_status, live_state_code, current_position_mm);
    } 

    // Защита от дубликатов: если пришел повтор токена и это не команда СТОП, то шлем DONE, не запуская мотор
    if (current_cmd_id == last_processed_cmd_id && can_pending_command != 0x03) { 

        extern int32_t current_position;

        int32_t pos_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);
        CAN_Network_SendExtendedStatus(current_cmd_id, 0x55, 0x02, pos_mm); 

        return;
    }
    
    // Триггер смены состояний (срабатывает 1 раз в момент окончания маневра)
    if (current_state != last_state) {
        uint8_t state_code = 0x04;
        uint8_t final_tx_status = 0x00;

        switch (current_state) {
            case STATE_MOVING:
            case STATE_HOMING:
                state_code = 0x01;
                final_tx_status = 0xAA;                     // Переключились в марш -> выдаем статус принятия команды (ACK)
                break;

            case STATE_READY:    
                state_code = 0x02; 
                final_tx_status = 0x55;                     // Выставляем DONE
                last_processed_cmd_id = current_cmd_id;     // Устанавливаем токен как успешно закрытый
                break;

            case STATE_FAULT:    
                state_code = 0x03; 
                final_tx_status = 0xFF; // Авария
                break; 

            case STATE_UNHOMED:  
                state_code = 0x04; 
                final_tx_status = 0xFF; 
                break;  

            default:             
                state_code = 0x04; 
                break; 
        }
        
        // Обновляем глобальный статус и отправляем кадр для ESP32
        transaction_status = final_tx_status;
        CAN_Network_SendExtendedStatus(current_cmd_id, transaction_status, state_code, current_position_mm);
        
        last_state = current_state;
    }
}

// Запускаем исполнение полученной команды
void CAN_Network_Update(void) {
    if (can_new_command_flag) {
        switch (can_pending_command) {
            case 0x01: Motion_StartHoming(); break;
            case 0x02: Motion_StartGoto(can_pending_target); break;
            case 0x03: Motion_StopExecution(); break;
            default:   break;
        }
        can_new_command_flag = 0; 
    }
}
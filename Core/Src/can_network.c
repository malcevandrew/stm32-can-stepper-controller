#include "can_network.h"
#include "motion.h"
#include <stdio.h>
#include <string.h>

// Указываем компилятору, что структура управления CAN (hcan) создана CubeMX в другом месте
extern CAN_HandleTypeDef hcan;

// Аппаратные структуры STM32 для работы с сообщениями
static CAN_TxHeaderTypeDef TxHeader;
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t RxData[8]; // Буфер для входящих байт данных (до 8 байт в кадре CAN)
static uint32_t TxMailbox;

// --- НАШ «ПОЧТОВЫЙ ЯЩИК» (БУФЕР КОМАНД) ---
volatile uint8_t can_new_command_flag = 0; 
volatile uint8_t can_pending_command = 0;  
volatile int32_t can_pending_target = 0;   


void CAN_Network_Init(void) {
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK; 
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 

    sFilterConfig.FilterIdHigh = (0x200 << 5); 
    sFilterConfig.FilterIdLow = 0x0000;

    sFilterConfig.FilterMaskIdHigh = (0x7FF << 5); 
    sFilterConfig.FilterMaskIdLow = 0x0000;

    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; 
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(&hcan) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    printf("[CAN] Node initialized. IDMask 0x7FF active. Listening only ID 0x200.\r\n");
}


// Аппаратный коллбек прерывания CAN
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_interface) {
    if (HAL_CAN_GetRxMessage(hcan_interface, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
        
        // Условие 1: Наш буфер команд свободен
        if (!can_new_command_flag && RxHeader.DLC > 0) {
            uint8_t incoming_cmd = RxData[0];
            uint8_t is_packet_valid = 0;

            // Условие 2: Сопоставление типа команды и длины кадра (DLC)
            if (incoming_cmd == 0x01 && RxHeader.DLC == 1) {    // Проверка на команду HOME (длина 1 байт)
                is_packet_valid = 1;
            } 
            else if (incoming_cmd == 0x02 && RxHeader.DLC >= 5) {   // Команда GOTO валидна (длина минимум 5 байт)
                
                uint32_t temp_target = ((uint32_t)RxData[1] << 24) |
                                       ((uint32_t)RxData[2] << 16) |
                                       ((uint32_t)RxData[3] << 8)  |
                                       ((uint32_t)RxData[4]);
                
                can_pending_target = (int32_t)temp_target;  //Получили значение цели в мм
                is_packet_valid = 1;
            }
            else if (incoming_cmd == 0x03 && RxHeader.DLC == 1) {   // Команда STOP валидна (длина 1 байт)
                
                is_packet_valid = 1;
            }

            // Только если пакет прошел сквозную валидацию, поднимаем рабочий флаг
            if (is_packet_valid) {
                can_pending_command = incoming_cmd;
                can_new_command_flag = 1;
            }
        }
    }
}


// Фоновый диспетчер выполнения команд
void CAN_Network_Update(void) {
    if (can_new_command_flag) {
        switch (can_pending_command) {
            case 0x01: 
                Motion_StartHoming();
                break;
                
            case 0x02: 
                Motion_StartGoto(can_pending_target);
                break;
                
            case 0x03: //  Прием фрейма экстренной остановки
                Motion_StopExecution();
                break;
                
            default:
                break;
        }
        can_new_command_flag = 0; 
    }
}


// Функция периодической отправки статуса (вызывается в main)
void CAN_Network_UpdateStatus(void) {
    extern volatile ControllerState_t current_state;
    extern int32_t current_position;
    extern const float STEPS_PER_MM; 

    static uint32_t last_can_send_tick = 0;
    static ControllerState_t last_state = STATE_STARTUP;
    
    uint32_t current_tick = HAL_GetTick(); 
    int32_t current_position_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);

    // 1. Периодическая отправка во время движения (каждые 50 мс)
    if (current_state == STATE_MOVING || current_state == STATE_HOMING) {
        if (current_tick - last_can_send_tick >= 50) {
            last_can_send_tick = current_tick;
            CAN_Network_SendStatus(0x01, current_position_mm); // MOVING
        }
    } 

    // 2. Мгновенная точечная отправка при изменении состояния автомата
    else if (current_state != last_state) {
        uint8_t status_code;
        switch (current_state) {
            case STATE_FAULT:    status_code = 0x03; break; // FAULT
            case STATE_UNHOMED:  status_code = 0x04; break; // UNHOMED
            case STATE_READY:    status_code = 0x02; break; // READY
            default:             status_code = 0x04; break; // По умолчанию UNHOMED
        }
        CAN_Network_SendStatus(status_code, current_position_mm);
    }

    last_state = current_state;
}

// Функция отправки РЕАЛЬНОГО статуса и координаты в ESP32
void CAN_Network_SendStatus(uint8_t state_code, int32_t position) {
    CAN_TxHeaderTypeDef local_tx_header;
    uint8_t local_tx_data[5];
    uint32_t local_tx_mailbox;

    // Конфигурируем кадр на отправку с ID = 0x300
    local_tx_header.StdId = 0x300;
    local_tx_header.RTR = CAN_RTR_DATA;
    local_tx_header.IDE = CAN_ID_STD;
    local_tx_header.DLC = 5; // 1 байт статус + 4 байта координата
    local_tx_header.TransmitGlobalTime = DISABLE;

    // Байт 0: Текущий статус (0x01 - MOVING, 0x02 - READY)
    local_tx_data[0] = state_code;

    // Байты 1..4: Координата в формате Big Endian
    local_tx_data[1] = (uint8_t)((position >> 24) & 0xFF);
    local_tx_data[2] = (uint8_t)((position >> 16) & 0xFF);
    local_tx_data[3] = (uint8_t)((position >> 8)  & 0xFF);
    local_tx_data[4] = (uint8_t)(position         & 0xFF);

    // Отправляем пакет в шину. Если буфер забит, функция просто пропустит кадр, не зависая
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) 
    {
        HAL_CAN_AddTxMessage(&hcan, &local_tx_header, local_tx_data, &local_tx_mailbox);
    }
}
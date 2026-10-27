#include "main.h"
#include "cli.h"
#include "motion.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern UART_HandleTypeDef huart1;

#define RX_BUF_SIZE 32
static uint8_t rx_buffer[RX_BUF_SIZE];
static uint8_t rx_index = 0;
uint8_t rx_data; // Буфер для одного байта из прерывания

// Перенаправление printf в UART1 для GCC
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

// Инициализация UART
void CLI_Init(void) {
    
    volatile uint32_t tmpreg = huart1.Instance->SR;     // Сброс аппаратного мусора линии
    tmpreg = huart1.Instance->DR;

    (void)tmpreg; // Защита от предупреждения компилятора об unused variable

    // Запускаем прерывание приема первого байта
    HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {

        extern volatile uint8_t current_state; 
        extern int32_t current_position;
        extern int32_t target_position;

        // Реагируем на '\n' (конец посылки)
        if (rx_data == '\n') {
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Закрываем строку
                
                if (strcmp((char*)rx_buffer, "HOME") == 0) {        //Сравниваем полученную строку
                    Motion_StartHoming();
                } 
                else if (strcmp((char*)rx_buffer, "STATUS") == 0) {
                    printf("\r\n--- STATUS ---\r\nState: %d\r\nPosition: %ld\r\nTarget: %ld\r\n--------------\r\n", 
                           current_state, current_position, target_position);
                } 
                else if (strncmp((char*)rx_buffer, "GOTO ", 5) == 0) {
                    int32_t parsed_target = atoi((char*)(rx_buffer + 5));
                    Motion_StartGoto(parsed_target);
                }
                else {
                    printf("\r\nERROR: Unknown command: %s\r\n", rx_buffer);
                }
                
                rx_index = 0; // Сбрасываем индекс для новой команды
            }
        } 
        // Игнорируем служебный '\r', а остальные символы берем в буфер
        else if (rx_data != '\r') {
            if (rx_index < RX_BUF_SIZE - 1) {
                rx_buffer[rx_index++] = rx_data;
            }
        }

        // Перезапускаем прерывание на прием следующего байта
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

// Коллбек ошибок линии UART
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1) {
        volatile uint32_t sr = huart->Instance->SR;
        volatile uint32_t dr = huart->Instance->DR;
        (void)sr; (void)dr;
        HAL_UART_Receive_IT(&huart1, &rx_data, 1);
    }
}

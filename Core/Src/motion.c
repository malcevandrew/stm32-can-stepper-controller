#include "motion.h"
#include "tim.h"
#include "can_network.h"
#include <stdlib.h>
#include <stdio.h>

extern volatile uint8_t current_cmd_id;
extern volatile uint8_t last_processed_cmd_id;
extern volatile uint8_t transaction_status;

volatile ControllerState_t current_state = STATE_STARTUP;   // Текущее состояние. При включении устанавливаем STATE_STARTUP
int32_t current_position = 0;                               // Текушая позиция в шагах мотора
int32_t target_position = 0;                                // Целевая координата в шагах мотора      
uint32_t steps_executed = 0;                                // Шагов выполнено
uint32_t total_move_distance = 0;                           // длина дистанции в шагах мотора
uint32_t current_period_us = START_PERIOD_US;               // 
int32_t max_position_steps = 0;
uint32_t homing_steps_counter = 0;
uint8_t homing_completed = 0;

// === ВОЗВРАЩАЕМ ВАШ РОДНОЙ РАБОЧИЙ ШАГОВЫЙ БЛОК ===
#define STEP_PORT GPIOB
static const uint16_t step_pins[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
static const uint8_t step_table[4][4] = {
    {1, 0, 0, 1}, 
    {0, 0, 1, 1},  
    {0, 1, 1, 0},  
    {1, 1, 0, 0}  
};
static int8_t current_step_index = 0;

// Инициализация 
void Motion_Init(void) {
    max_position_steps = (int32_t)((float)MAX_POSITION_MM * STEPS_PER_MM);
    current_state = STATE_UNHOMED;
}

// Выполнение 1 шага мотора
void Motion_Step(int8_t direction) {

    current_step_index += direction;
    if (current_step_index > 3) current_step_index = 0;
    if (current_step_index < 0) current_step_index = 3;

    // Подаем сигнал на обмотки шагового мотора
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(STEP_PORT, step_pins[i], step_table[current_step_index][i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

// При получении команды HOME инициализируем процесс калибровки
void Motion_StartHoming(void) {
    int32_t current_pos_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);

    if (current_state != STATE_MOVING && current_state != STATE_HOMING) {
        homing_steps_counter = 0; 
        current_period_us = 4000; 
        __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);    // Устанавливаем пониженную скорость (период таймера 4000)
        
        transaction_status = 0x00; 
        current_state = STATE_HOMING;
    } else {
        CAN_Network_SendExtendedStatus(current_cmd_id, 0xFF, 0x01, current_pos_mm);
    }
}

// При получении команды GoТo инициализируем 
void Motion_StartGoto(int32_t new_target_mm) {
    int32_t current_pos_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);

    if (current_state != STATE_READY && current_state != STATE_MOVING) {
        transaction_status = 0xFF; 
        CAN_Network_SendExtendedStatus(current_cmd_id, transaction_status, 0x04, current_pos_mm); 
        return; 
    }

    if (new_target_mm < MIN_POSITION_MM || new_target_mm > MAX_POSITION_MM) {
        current_state = STATE_FAULT;
        transaction_status = 0xFF; 
        CAN_Network_SendExtendedStatus(current_cmd_id, transaction_status, 0x03, current_pos_mm); 
        return; 
    }
    
    transaction_status = 0xAA; 
    target_position = (int32_t)(((float)new_target_mm * STEPS_PER_MM) + 0.5f); 
    steps_executed = 0;
    current_period_us = START_PERIOD_US;
    total_move_distance = abs(target_position - current_position); 
    
    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);    // 
    current_state = STATE_MOVING;

    CAN_Network_SendExtendedStatus(current_cmd_id, transaction_status, 0x01, current_pos_mm);   // Сообщаем что 
}

void Motion_StopExecution(void) {
    if (current_state == STATE_MOVING || current_state == STATE_HOMING) {
        uint8_t was_homing = (current_state == STATE_HOMING);

        target_position = current_position; 
        steps_executed = 0;
        total_move_distance = 0;
        current_period_us = START_PERIOD_US;
        __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
        
        current_state = was_homing ? STATE_UNHOMED : STATE_READY;
        
        transaction_status = 0xFF; 
        int32_t current_pos_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);
        CAN_Network_SendExtendedStatus(current_cmd_id, transaction_status, (was_homing ? 0x04 : 0x02), current_pos_mm);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        
        switch (current_state) {
            
            case STATE_HOMING:
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) { 
                    current_position = 0;
                    target_position = 0;
                    homing_steps_counter = 0;
                    homing_completed = 1;
                    

                    current_state = STATE_READY; // Переключаем автомат
                } else {
                    Motion_Step(-1);

                    //Обработка случая, если датчик нуля не сработает
                    if (++homing_steps_counter >= (uint32_t)max_position_steps) {  
                        current_state = STATE_FAULT;
                        homing_steps_counter = 0;
                    }
                }
                break;
                
            case STATE_MOVING:
                if (current_position != target_position) {
                    int8_t dir = (target_position > current_position) ? 1 : -1;
                    
                    if (total_move_distance < ACCEL_LIMIT_STEPS) {
                        current_period_us = START_PERIOD_US;
                    } else {
                        uint32_t decel_start_step;
                        uint32_t local_accel_limit;

                        if (total_move_distance < (ACCEL_LIMIT_STEPS * 2)) {
                            decel_start_step = total_move_distance / 2;
                            local_accel_limit = decel_start_step;
                        } else {
                            decel_start_step = total_move_distance - ACCEL_LIMIT_STEPS;
                            local_accel_limit = ACCEL_LIMIT_STEPS;
                        }
                        
                        // разгон
                        if (steps_executed < local_accel_limit) {   
                            current_period_us = START_PERIOD_US - ((START_PERIOD_US - MAX_SPEED_PERIOD_US) * steps_executed / local_accel_limit);
                        // торможение    
                        } else if (steps_executed >= decel_start_step) {    
                            uint32_t steps_from_decel_start = steps_executed - decel_start_step;
                            uint32_t decel_range = total_move_distance - decel_start_step;
                            current_period_us = MAX_SPEED_PERIOD_US + ((START_PERIOD_US - MAX_SPEED_PERIOD_US) * steps_from_decel_start / decel_range);
                        // марш
                        } else {    
                            current_period_us = MAX_SPEED_PERIOD_US;
                        }
                    }

                    if (current_period_us < MAX_SPEED_PERIOD_US) current_period_us = MAX_SPEED_PERIOD_US;
                    if (current_period_us > START_PERIOD_US) current_period_us = START_PERIOD_US;

                    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
                    Motion_Step(dir);
                    current_position += dir; // Координата обновляется СТРОГО здесь
                    steps_executed++;
                    
                } else {
                    current_state = STATE_READY;
                    current_period_us = START_PERIOD_US;
                    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
                       
                    //int32_t final_pos_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);
                     
                }
                break;
                
            default:
                break;
        }
    }
}
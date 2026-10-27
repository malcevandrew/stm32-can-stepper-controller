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
uint32_t total_move_distance = 0;                           // Длина дистанции у принятой команды в шагах мотора
uint32_t current_period_us = START_PERIOD_US;               // Текущее значение периода таймера (мс) - скорость вращения мотора
int32_t max_position_steps = 0;                             // Предел движения, переведенный в кол-во шагов мотора
uint32_t homing_steps_counter = 0;                          // Подсчет шагов во время калибровки (для защиты, если датчик нуля не сработает)
uint8_t homing_completed = 0;                               // Статус калибровки

// Шаблон управления обкладками шагового двигателя
#define STEP_PORT GPIOB
static const uint16_t step_pins[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
static const uint8_t step_table[4][4] = {
    {1, 0, 0, 1}, 
    {0, 0, 1, 1},  
    {0, 1, 1, 0},  
    {1, 1, 0, 0}  
};
static int8_t current_step_index = 0;

// Инициализация после включения
void Motion_Init(void) {
    max_position_steps = (int32_t)((float)MAX_POSITION_MM * STEPS_PER_MM);  // Расчет предела движения для данной установки
    current_state = STATE_UNHOMED;
}

// Выполнение одного шага мотора в нужном направлении
void Motion_Step(int8_t direction) {

    current_step_index += direction;
    if (current_step_index > 3) current_step_index = 0;
    if (current_step_index < 0) current_step_index = 3;

    // Подаем сигнал на обмотки шагового мотора по шаблону
    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(STEP_PORT, step_pins[i], step_table[current_step_index][i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

// Инициализация процесса калибровки при получении команды HOME
void Motion_StartHoming(void) {
    int32_t current_pos_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);

    if (current_state != STATE_MOVING && current_state != STATE_HOMING) {
        homing_steps_counter = 0; 
        current_period_us = 4000; 
        __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);    // Устанавливаем пониженную скорость (период таймера 4000)
        
        transaction_status = 0x00; 
        current_state = STATE_HOMING;
    } else {
        CAN_Network_SendExtendedStatus(current_cmd_id, 0xFF, 0x01, current_pos_mm); // Если мотор уже в процессе движения или калибровки отправляем NASK
    }
}

// Подготовка к началу движения при получении команды GoТo 
void Motion_StartGoto(int32_t new_target_mm) {
    int32_t current_pos_mm = (int32_t)(((float)current_position / STEPS_PER_MM) + 0.5f);

    // Проверка была ли калибровка
    if (current_state != STATE_READY && current_state != STATE_MOVING) {
        transaction_status = 0xFF; 
        CAN_Network_SendExtendedStatus(current_cmd_id, transaction_status, 0x04, current_pos_mm); 
        return; 
    }

    // Входит ли принятая команда в установленные пределы
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
    
    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);    // Устанавливаем период таймера (начальную скорость мотора)
    current_state = STATE_MOVING;

    CAN_Network_SendExtendedStatus(current_cmd_id, transaction_status, 0x01, current_pos_mm);   // Отправляем первый кадр со статусом ASK 
}

// При получении команды Stop 
void Motion_StopExecution(void) {
    if (current_state == STATE_MOVING || current_state == STATE_HOMING) {
        uint8_t was_homing = (current_state == STATE_HOMING);

        target_position = current_position;         // Просто устанавливаем текущую позицию как целевую
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

// Коллбек прерывания по переполнению таймера 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        
        switch (current_state) {
            
            // Движение с постоянной скоростью до момента срабатывания фотопрерывателя
            case STATE_HOMING:
                // Если фотопрерыватель сработал, то фиксируем координату 0 мм
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {      
                    current_position = 0;
                    target_position = 0;
                    homing_steps_counter = 0;
                    homing_completed = 1;
                    current_state = STATE_READY; // Переключаем автомат
                // Иначе продожаем движение в сторону нуля
                } else {
                    Motion_Step(-1);

                    // Защита на случай, если датчик нуля не сработает
                    if (++homing_steps_counter >= (uint32_t)max_position_steps) {  
                        current_state = STATE_FAULT;
                        homing_steps_counter = 0;
                    }
                }
                break;
                
            // Расчет периода таймера для следующего шага    
            case STATE_MOVING:
                if (current_position != target_position) {
                    int8_t dir = (target_position > current_position) ? 1 : -1;
                    
                    // Если кол-во шагов до целевой координаты меньше кол-ва шагов для разгона, то просто движемся с начальной скоростью
                    if (total_move_distance < ACCEL_LIMIT_STEPS) {
                        current_period_us = START_PERIOD_US;
                    // Иначе будем изменять скорости по профилю трапеции (разгон - марш - торможение)
                    } else {
                        uint32_t decel_start_step;  // Шаг начала торможения

                        
                        uint32_t local_accel_limit;                             // Для дистанций от ACCEL_LIMIT_STEPS до (ACCEL_LIMIT_STEPS * 2),          
                        if (total_move_distance < (ACCEL_LIMIT_STEPS * 2)) {    // будем сокращать кол-ва шагов разгона и торможения
                            decel_start_step = total_move_distance / 2;
                            local_accel_limit = decel_start_step;
                        } else {
                            decel_start_step = total_move_distance - ACCEL_LIMIT_STEPS; // Для больших дистанций
                            local_accel_limit = ACCEL_LIMIT_STEPS;
                        }
                        
                        // Расчитываем скорость (период таймера) для следующего шага в зависимости от текущей позиции
                        // Участок разгона
                        if (steps_executed < local_accel_limit) {   
                            current_period_us = START_PERIOD_US - ((START_PERIOD_US - MAX_SPEED_PERIOD_US) * steps_executed / local_accel_limit);
                        // Участок торможения   
                        } else if (steps_executed >= decel_start_step) {    
                            uint32_t steps_from_decel_start = steps_executed - decel_start_step;
                            uint32_t decel_range = total_move_distance - decel_start_step;
                            current_period_us = MAX_SPEED_PERIOD_US + ((START_PERIOD_US - MAX_SPEED_PERIOD_US) * steps_from_decel_start / decel_range);
                        // Участок максимальной скорости (марш)
                        } else {    
                            current_period_us = MAX_SPEED_PERIOD_US;
                        }
                    }

                    // Защита для выхода за границы установленных пределов скорости
                    if (current_period_us < MAX_SPEED_PERIOD_US) current_period_us = MAX_SPEED_PERIOD_US;
                    if (current_period_us > START_PERIOD_US) current_period_us = START_PERIOD_US;

                    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);    // Перезаписываем период таймера

                    Motion_Step(dir);                   // Выпоняем один шаг
                    current_position += dir;            // Обновляем координату
                    steps_executed++;                   
                    
                } else {
                    current_state = STATE_READY;
                    current_period_us = START_PERIOD_US;
                    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
                }
                break;
                
            default:
                break;
        }
    }
}
#include "motion.h"
#include "tim.h"
#include <stdlib.h>
#include <stdio.h>

// Определение глобальных переменных

// Переменные профиля скорости (в микросекундах периода таймера)
const uint32_t START_PERIOD_US = 6000;      // Стартовая скорость (медленно)
const uint32_t MAX_SPEED_PERIOD_US = 1500;  // Максимальная маршевая скорость (быстро)
const uint32_t ACCEL_LIMIT_STEPS = 450;     // Дистанция разгона/торможения (в шагах)

uint32_t current_period_us = 8000;          // Текущий период (изменяется на лету)
uint32_t steps_executed = 0;                // Сколько шагов сделано с начала текущего движения
uint32_t total_move_distance = 0;  

volatile ControllerState_t current_state = STATE_STARTUP;
int32_t current_position = 0;
int32_t target_position = 0;

// Настройки перевода шагов в миллиметры для 28BYJ-48 (Вал ~18 мм)
const float STEPS_PER_MM = 44.15f; 

const int32_t MIN_POSITION = 0;    // 0 мм
const int32_t MAX_POSITION = 200; // 200 мм == 20см

// Локальные переменные драйвера мотора
#define STEP_PORT GPIOB
static const uint16_t step_pins[] = {GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15};
static const uint8_t step_table[4][4] = {
    {1, 0, 0, 1}, 
    {0, 0, 1, 1},  
    {0, 1, 1, 0},  
    {1, 1, 0, 0}  
};
static int8_t current_step_index = 0;

volatile uint8_t homing_completed = 0;  

uint32_t homing_steps_counter = 0;

// лимит в ФИЗИЧЕСКИХ ШАГАХ
int32_t max_position_steps = 0; 


void Motion_Init(void) {
    
    max_position_steps = (int32_t)((float)MAX_POSITION * STEPS_PER_MM); // Считаем лимит в шагах ОДИН раз при включении платы
    
    current_state = STATE_UNHOMED;
}




// Физическое переключение обмоток в зависимости от направления движения
void Motion_Step(int8_t direction) {
    current_step_index += direction;
    if (current_step_index > 3) current_step_index = 0;
    if (current_step_index < 0) current_step_index = 3;

    for (int i = 0; i < 4; i++) {
        HAL_GPIO_WritePin(STEP_PORT, step_pins[i], step_table[current_step_index][i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    current_position += direction;  // обновляем текущее положение 
}

// Запуск процедуры поиска нуля
void Motion_StartHoming(void) {
    if (current_state == STATE_UNHOMED || current_state == STATE_READY || current_state == STATE_FAULT) {   
        printf("\r\nAction: Starting Homing...\r\n");
        
        homing_steps_counter = 0;   // Cбрасываем счетчик шагов поиска 0 при каждом новом старте калибровки
        current_period_us = 4000;   // 3адаем безопасный период таймера для поиска нуля (4000 мкс)

        __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
        
        current_state = STATE_HOMING;
    } else {                                    
        printf("\r\nERROR: Device Busy\r\n");
    }
}

void Motion_StartGoto(int32_t new_target_mm) {
    if (current_state != STATE_READY && current_state != STATE_MOVING) {
        printf("\r\nERROR: Device is UNHOMED or in FAULT state.\r\n");
        return;
    }
    // Проверка лимитов идет в миллиметрах
    if (new_target_mm < MIN_POSITION || new_target_mm > MAX_POSITION) {
        printf("\r\nERROR: POSITION_LIMIT (%ld..%ld mm)\r\n", MIN_POSITION, MAX_POSITION);
        return;
    }
    
    // Переводим миллиметры из команды в физические шаги редуктора + округляем к ближайшему целому шагу редуктора
    target_position = (int32_t)(((float)new_target_mm * STEPS_PER_MM) + 0.5f);
    steps_executed = 0;
    current_period_us = START_PERIOD_US;
    
    // Вычисляем дистанцию перемещения в физических шагах
    total_move_distance = abs(target_position - current_position); 
    
    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
    current_state = STATE_MOVING;
    printf("\r\nMoving to absolute position: %ld mm (Target steps: %ld)...\r\n", new_target_mm, target_position);
}




void Motion_Update(void) {       // Функция для отслеживания изменений

    static ControllerState_t last_printed_state = STATE_STARTUP;    

    if (homing_completed) {
        printf("\r\nHOMING OK. State: READY\r\n");
        homing_completed = 0; // Сбрасываем флаг
    }


    // Защита: если состояние перешло в FAULT из-за таймаута в прерывании
    if (current_state == STATE_FAULT && last_printed_state != STATE_FAULT) {
        printf("\r\nERROR: HOMING TIMEOUT! Axis limit reached. State: FAULT\r\n");
        last_printed_state = STATE_FAULT;
    }

    // Сообщаем в UART, когда каретка успешно доехала до цели
    if (current_state == STATE_READY && last_printed_state == STATE_MOVING) {
        printf("\r\n[Motion] Target reached! Position: %ld. State: READY\r\n", current_position);
        last_printed_state = STATE_READY;
    }

    // Запоминаем текущее состояние для отслеживания изменений
    if (current_state != last_printed_state) {
        last_printed_state = current_state;
    }
}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {       
    
    // Обработка прерывания именно от TIM2. Вызывается когда счетчик досчитал до текущего значения периода таймера
    if (htim->Instance == TIM2) {
        
        switch (current_state) {
            case STATE_STARTUP:
            case STATE_UNHOMED:
            case STATE_READY:
            case STATE_FAULT:
                break;

            case STATE_HOMING:      // Едем назад (-1) с постоянной безопасной скоростью
                
                //Движемся пока флаг на каретке не перекроет фотопрерыватель
                if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET) {   
                    current_position = 0;
                    target_position = 0;
                    current_state = STATE_READY;
                    homing_completed = 1;
                } else {
                    Motion_Step(-1);
                    
                    //Проверка на случай если фотопрерыватель не сработал
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
                        // Сверхкороткий ход: едем монотонно на безопасной стартовой скорости
                        current_period_us = START_PERIOD_US;
                    } else {
                        uint32_t decel_start_step;
                        uint32_t local_accel_limit;

                        // Математическое разделение коротких и средних дистанций перемещения
                        if (total_move_distance < (ACCEL_LIMIT_STEPS * 2)) {
                            decel_start_step = total_move_distance / 2; // Тормозим ровно с середины пути
                            local_accel_limit = decel_start_step;
                        } else {
                            decel_start_step = total_move_distance - ACCEL_LIMIT_STEPS;
                            local_accel_limit = ACCEL_LIMIT_STEPS;
                        }
                        
                        // Просчет фазы профиля скорости
                        if (steps_executed < local_accel_limit) {
                            // Фаза 1: Линейный разгон
                            current_period_us = START_PERIOD_US - ((START_PERIOD_US - MAX_SPEED_PERIOD_US) * steps_executed / local_accel_limit);
                        } else if (steps_executed >= decel_start_step) {
                            // Фаза 3: Линейное торможение
                            uint32_t steps_from_decel_start = steps_executed - decel_start_step;
                            uint32_t decel_range = total_move_distance - decel_start_step;
                            current_period_us = MAX_SPEED_PERIOD_US + ((START_PERIOD_US - MAX_SPEED_PERIOD_US) * steps_from_decel_start / decel_range);
                        } else {
                            // Фаза 2: Марш на максимальной скорости
                            current_period_us = MAX_SPEED_PERIOD_US;
                        }
                    }

                    if (current_period_us < MAX_SPEED_PERIOD_US) current_period_us = MAX_SPEED_PERIOD_US;
                    if (current_period_us > START_PERIOD_US) current_period_us = START_PERIOD_US;

                    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
                    Motion_Step(dir);
                    steps_executed++;
                    
                } else {
                    current_state = STATE_READY;
                    current_period_us = START_PERIOD_US;
                    __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
                }
                break;

        }
    }
}


void Motion_StopExecution(void) {
    if (current_state == STATE_MOVING || current_state == STATE_HOMING) {
        
        // Запоминаем, прервали ли мы калибровку нуля
        uint8_t was_homing = (current_state == STATE_HOMING);

        target_position = current_position; 
        steps_executed = 0;
        total_move_distance = 0;
        
        current_period_us = START_PERIOD_US;
        __HAL_TIM_SET_AUTORELOAD(&htim2, current_period_us);
        
        // Если стоп нажали во время калибровки — возвращаемся в UNHOMED!
        if (was_homing) {
            current_state = STATE_UNHOMED;
            printf("\r\n[CAN] HOME interrupted by STOP. Device reset to UNHOMED.\r\n");
        } else {
            current_state = STATE_READY;
            printf("\r\n[CAN] GOTO interrupted by STOP. Position locked.\r\n");
        }
    }
}

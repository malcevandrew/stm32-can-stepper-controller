#ifndef __MOTION_H
#define __MOTION_H

#include "main.h"

// Перечисление состояний системы (State Machine) из ТЗ
typedef enum {
    STATE_STARTUP,
    STATE_UNHOMED,      //
    STATE_HOMING,
    STATE_READY,
    STATE_MOVING,
    STATE_FAULT
} ControllerState_t;

// Глобальные переменные, доступные для CLI и main
extern volatile ControllerState_t current_state;
extern int32_t current_position;
extern int32_t target_position;
extern uint32_t steps_executed;

// Константы ограничений
extern const int32_t MIN_POSITION;
extern const int32_t MAX_POSITION;

// Прототипы функций модулей
void Motion_Init(void);
void Motion_Step(int8_t direction);
void Motion_Update(void);
void Motion_StartHoming(void);
void Motion_StartGoto(int32_t new_target);
void Motion_StopExecution(void);

#endif /* __MOTION_H */

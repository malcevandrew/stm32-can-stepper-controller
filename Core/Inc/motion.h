#ifndef MOTION_H
#define MOTION_H

#include "stm32f1xx_hal.h"

// ============================================================================
//                   КОНФИГУРАЦИЯ УСТАНОВКИ
// ============================================================================
#define STEPS_PER_MM            44.15f    // Коэфициент перевода шагов мотора в мм (44.15 шаг/мм)
#define MIN_POSITION_MM         0         // Физический ноль при достижении концевика(фотопрерывателя)
#define MAX_POSITION_MM         170       // Предел хода каретки (17 см)

#define START_PERIOD_US         6000      // Стартовая скорость
#define MAX_SPEED_PERIOD_US     1500      // Маршевая максимальная скорость
#define ACCEL_LIMIT_STEPS       450       // Количество шагов мотора для разгона и торможения
// ============================================================================

typedef enum {
    STATE_STARTUP = 0,
    STATE_UNHOMED,
    STATE_HOMING,
    STATE_READY,
    STATE_MOVING,
    STATE_FAULT
} ControllerState_t;

void Motion_Init(void);
void Motion_StartHoming(void);
void Motion_StartGoto(int32_t new_target_mm);
void Motion_StopExecution(void);
void Motion_Step(int8_t direction);

#endif // MOTION_H
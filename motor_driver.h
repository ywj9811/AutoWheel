#ifndef _MOTOR_DRIVER_H_
#define _MOTOR_DRIVER_H_

#include <stdint.h>

typedef enum {
    MOTOR_DIR_FORD = 0,
    MOTOR_DIR_RIGH,
    MOTOR_DIR_LEFT,
    MOTOR_DIR_BACK,
    MOTOR_DIR_STOP,
    MOTOR_DIR_END
}motor_direction_e; // motor_set_direction 여기에 어떤 방향으로 가는지 넣어줄 enum값들임

void motor_allStop();
void motor_stop();
void motor_left();
void motor_right();
void motor_streight();
void motor_back();
int motor_init();
void* motor_run(void *arg);
void motor_set_direction(motor_direction_e direction);

extern uint8_t motor_direction;

#endif
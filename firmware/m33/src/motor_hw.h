/* Drives the L298N: two TPM PWM channels for speed, four GPIOs for direction. */
#ifndef HERO_MOTOR_HW_H
#define HERO_MOTOR_HW_H

#include <stdint.h>

void motor_hw_init(void);                  /* pins muxed, PWM at 0 %, all INx low */
void motor_hw_apply(int8_t left, int8_t right);   /* -100..100 % */

#endif

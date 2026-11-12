/*
 * Hero wiring on the FRDM-IMX93 40-pin header (Raspberry Pi layout:
 * header pin N carries GPIO_IO<BCM number>).
 *
 *   L298N   header pin   pad          M33 function
 *   IN1     3            GPIO_IO02    GPIO2_IO02   left motor, reverse
 *   IN2     5            GPIO_IO03    GPIO2_IO03   left motor, forward
 *   IN3     7            GPIO_IO04    GPIO2_IO04   right motor, reverse
 *   IN4     8            GPIO_IO14    GPIO2_IO14   right motor, forward
 *   ENA     32           GPIO_IO12    TPM3_CH2     left motor PWM
 *   ENB     33           GPIO_IO13    TPM4_CH2     right motor PWM
 *
 * "Forward = IN2 / IN4 high" matches how the motors are wired on Hero.
 * GPIO_IO12/13 also drive the board's RGB LED, so the LED glows with motor speed.
 * Remove the ENA/ENB jumpers on the L298N before connecting pins 32/33.
 */
#ifndef HERO_BOARD_H
#define HERO_BOARD_H

#define HERO_GPIO           GPIO2
#define HERO_PIN_IN1        2U
#define HERO_PIN_IN2        3U
#define HERO_PIN_IN3        4U
#define HERO_PIN_IN4        14U

#define HERO_PWM_LEFT_TPM   TPM3
#define HERO_PWM_LEFT_CH    kTPM_Chnl_2
#define HERO_PWM_RIGHT_TPM  TPM4
#define HERO_PWM_RIGHT_CH   kTPM_Chnl_2
#define HERO_PWM_HZ         20000U      /* above hearing range; L298N handles up to ~40 kHz */

/* RPMsg: Linux's imx_rpmsg_tty binds to this name and creates /dev/ttyRPMSG<addr>. */
#define HERO_EPT_ADDR       30U
#define HERO_NS_NAME        "rpmsg-virtual-tty-channel"
#define HERO_FW_VERSION     "hero-m33 1.0.0"

#endif

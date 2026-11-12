#include "motor_hw.h"
#include "hero_board.h"

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "fsl_rgpio.h"
#include "fsl_tpm.h"

/* Pad settings: drive strength X4, slew fast, no pull (same as Linux used, 0x31e). */
#define PAD_CFG 0x31EU

static void pins_init(void)
{
    IOMUXC_SetPinMux(IOMUXC_PAD_GPIO_IO02__GPIO2_IO02, 0U);
    IOMUXC_SetPinMux(IOMUXC_PAD_GPIO_IO03__GPIO2_IO03, 0U);
    IOMUXC_SetPinMux(IOMUXC_PAD_GPIO_IO04__GPIO2_IO04, 0U);
    IOMUXC_SetPinMux(IOMUXC_PAD_GPIO_IO14__GPIO2_IO14, 0U);
    IOMUXC_SetPinMux(IOMUXC_PAD_GPIO_IO12__TPM3_CH2, 0U);
    IOMUXC_SetPinMux(IOMUXC_PAD_GPIO_IO13__TPM4_CH2, 0U);

    IOMUXC_SetPinConfig(IOMUXC_PAD_GPIO_IO02__GPIO2_IO02, PAD_CFG);
    IOMUXC_SetPinConfig(IOMUXC_PAD_GPIO_IO03__GPIO2_IO03, PAD_CFG);
    IOMUXC_SetPinConfig(IOMUXC_PAD_GPIO_IO04__GPIO2_IO04, PAD_CFG);
    IOMUXC_SetPinConfig(IOMUXC_PAD_GPIO_IO14__GPIO2_IO14, PAD_CFG);
    IOMUXC_SetPinConfig(IOMUXC_PAD_GPIO_IO12__TPM3_CH2, PAD_CFG);
    IOMUXC_SetPinConfig(IOMUXC_PAD_GPIO_IO13__TPM4_CH2, PAD_CFG);

    rgpio_pin_config_t out_low = { kRGPIO_DigitalOutput, 0U };
    RGPIO_PinInit(HERO_GPIO, HERO_PIN_IN1, &out_low);
    RGPIO_PinInit(HERO_GPIO, HERO_PIN_IN2, &out_low);
    RGPIO_PinInit(HERO_GPIO, HERO_PIN_IN3, &out_low);
    RGPIO_PinInit(HERO_GPIO, HERO_PIN_IN4, &out_low);
}

static void pwm_init(TPM_Type *tpm, tpm_chnl_t ch, clock_root_t root, uint8_t mux_osc24m)
{
    /* 24 MHz oscillator into the TPM keeps the 20 kHz period exact */
    const clock_root_config_t cfg = { .clockOff = false, .mux = mux_osc24m, .div = 1 };
    CLOCK_SetRootClock(root, &cfg);

    tpm_config_t tc;
    TPM_GetDefaultConfig(&tc);
    TPM_Init(tpm, &tc);

    tpm_chnl_pwm_signal_param_t p = {
        .chnlNumber = ch,
        .level = kTPM_HighTrue,
        .dutyCyclePercent = 0U,
    };
    TPM_SetupPwm(tpm, &p, 1U, kTPM_EdgeAlignedPwm, HERO_PWM_HZ, CLOCK_GetIpFreq(root));
    TPM_StartTimer(tpm, kTPM_SystemClock);
}

void motor_hw_init(void)
{
    pins_init();
    pwm_init(HERO_PWM_LEFT_TPM, HERO_PWM_LEFT_CH, kCLOCK_Root_Tpm3, kCLOCK_TPM3_ClockRoot_MuxOsc24M);
    pwm_init(HERO_PWM_RIGHT_TPM, HERO_PWM_RIGHT_CH, kCLOCK_Root_Tpm4, kCLOCK_TPM4_ClockRoot_MuxOsc24M);
}

/* One side: direction pins first, then duty. 0 % = both INx low (coast). */
static void side(TPM_Type *tpm, tpm_chnl_t ch, uint32_t pin_rev, uint32_t pin_fwd, int8_t duty)
{
    uint8_t pct = (uint8_t)(duty < 0 ? -duty : duty);
    RGPIO_PinWrite(HERO_GPIO, pin_fwd, duty > 0 ? 1U : 0U);
    RGPIO_PinWrite(HERO_GPIO, pin_rev, duty < 0 ? 1U : 0U);
    TPM_UpdatePwmDutycycle(tpm, ch, kTPM_EdgeAlignedPwm, pct);
}

void motor_hw_apply(int8_t left, int8_t right)
{
    side(HERO_PWM_LEFT_TPM, HERO_PWM_LEFT_CH, HERO_PIN_IN1, HERO_PIN_IN2, left);
    side(HERO_PWM_RIGHT_TPM, HERO_PWM_RIGHT_CH, HERO_PIN_IN3, HERO_PIN_IN4, right);
}

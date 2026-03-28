
#include "modulo.h"
#include "driver/ledc.h"

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_PERIOD_US    20000 // 1/50Hz = 20ms


void init_pwm_servoMotor(ledc_mode_t Led_mode, ledc_channel_t channel_led, ledc_timer_t timer_led, gpio_num_t GPIO_pin, int FREQ_timer, ledc_timer_bit_t Duty_res)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = Led_mode,
        .duty_resolution  = Duty_res,
        .timer_num        = timer_led,
        .freq_hz          = FREQ_timer,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = Led_mode,
        .channel        = channel_led,
        .timer_sel      = timer_led,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = GPIO_pin,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

uint32_t angle_to_duty(int angle)
{
    int pulse = SERVO_MIN_PULSE_US + (angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180); //miliseconds
    uint32_t duty = (pulse * ((1 << 16) - 1)) / SERVO_PERIOD_US; //resolution on binary
    return duty;
}


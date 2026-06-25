
#include "driver/ledc.h"
#include "driver/gpio.h" //<---- Now its separate module. ESP-IDF 6.2v

void init_pwm_servoMotor(ledc_mode_t Led_mode, ledc_channel_t channel_led, ledc_timer_t timer_led, gpio_num_t GPIO_pin, int FREQ_timer, ledc_timer_bit_t Duty_res);
uint32_t angle_to_duty(int angle);

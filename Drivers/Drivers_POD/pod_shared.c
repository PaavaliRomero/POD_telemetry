
#include "pod_shared.h"

volatile pod_state_enum pod_state = POD_STATE_INIT;
volatile UI_state_enum pod_state_ui = POD_STATE_AnimaLoad;

volatile uint16_t VBATT = 0;
volatile uint8_t State_LED = 0;

volatile uint8_t  imu_active  = 0;
uint32_t last_activity_tick = 0;


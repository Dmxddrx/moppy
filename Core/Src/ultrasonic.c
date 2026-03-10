#include "ultrasonic.h"

static UltrasonicSensor sensors[NUM_ULTRASONIC];
static TIM_HandleTypeDef *htim;

void US_Init(TIM_HandleTypeDef *ht)
{
    htim = ht;

    for(int i=0;i<NUM_ULTRASONIC;i++)
        sensors[i].distance_mm = 0;
}

// Trigger function (send 10us pulse on TRIG pin)
void US_Trigger(uint8_t sensor_index)
{
    // Implement GPIO high-low for TRIG pins here
    // Example: HAL_GPIO_WritePin(TRIG_PORT[sensor_index], TRIG_PIN[sensor_index], GPIO_PIN_SET);
    // Delay 10us
    // HAL_GPIO_WritePin(..., GPIO_PIN_RESET);
}

// Call periodically in main loop or timer
void US_Update(void)
{
    // Read TIM2 CCR1-4 capture registers
    // Calculate distance_mm = (CCR / timer_freq) * 34300 / 2
    // Fill sensors[i].distance_mm
}

uint32_t US_GetDistance(uint8_t sensor_index)
{
    if(sensor_index >= NUM_ULTRASONIC) return 0;
    return sensors[sensor_index].distance_mm;
}

#include "ir.h"

static IRSensor ir_sensors[NUM_IR];

void IR_Init(void)
{
    // Configure EXTI lines PD0, PD1, PD3, PD4 in CubeMX
    for(int i=0;i<NUM_IR;i++)
        ir_sensors[i].state = 0;
}

// Call in EXTI IRQ handler
void IR_Update(uint8_t sensor_index, uint8_t value)
{
    if(sensor_index < NUM_IR)
        ir_sensors[sensor_index].state = value;
}

uint8_t IR_Get(uint8_t sensor_index)
{
    if(sensor_index >= NUM_IR) return 0;
    return ir_sensors[sensor_index].state;
}

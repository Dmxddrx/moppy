#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>

#define MAP_SIZE_X      100
#define MAP_SIZE_Y      100
#define MAP_RESOLUTION  0.05f   /* metres per cell (5cm) */

#define CELL_UNKNOWN   0
#define CELL_FREE      1
#define CELL_OCCUPIED  2

/* ── Ultrasonic sensor mounting angles (degrees, robot-relative)
   Adjust to match your physical sensor placement              */
#define US_ANGLE_0   0.0f    /* front        */
#define US_ANGLE_1   90.0f   /* right        */
#define US_ANGLE_2   180.0f  /* rear         */
#define US_ANGLE_3   270.0f  /* left         */

/* ── IR sensor mounting angles (degrees, robot-relative)
   IR range is short — marks CELL_OCCUPIED at IR_RANGE_M      */
#define IR_ANGLE_0   45.0f   /* front-left   */
#define IR_ANGLE_1   135.0f  /* rear-left    */
#define IR_ANGLE_2   225.0f  /* rear-right   */
#define IR_ANGLE_3   315.0f  /* front-right  */
#define IR_RANGE_M   0.10f   /* 10cm fixed range for IR hit    */

typedef struct
{
    int8_t grid[MAP_SIZE_X][MAP_SIZE_Y];
    float  robot_x;
    float  robot_y;
    float  robot_theta;
} Map;

void    Map_Init(Map *map);
void    Map_UpdateRobotPose(Map *map, float x, float y, float theta);
void    Map_UpdateUltrasonic(Map *map, float distance, float sensor_angle_deg);
//void    Map_UpdateIR(Map *map, uint8_t sensor_index, uint8_t triggered);
uint8_t Map_GetCell(Map *map, int x, int y);

#endif

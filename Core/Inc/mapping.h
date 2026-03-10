#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>

#define MAP_SIZE_X 100
#define MAP_SIZE_Y 100
#define MAP_RESOLUTION 0.05f   // meters per cell (5cm)

#define CELL_UNKNOWN 0
#define CELL_FREE    1
#define CELL_OCCUPIED 2

typedef struct
{
    int8_t grid[MAP_SIZE_X][MAP_SIZE_Y];

    float robot_x;
    float robot_y;
    float robot_theta;

} Map;

void Map_Init(Map *map);

void Map_UpdateRobotPose(Map *map, float x, float y, float theta);

void Map_UpdateUltrasonic(Map *map, float distance, float sensor_angle);

uint8_t Map_GetCell(Map *map, int x, int y);

#endif

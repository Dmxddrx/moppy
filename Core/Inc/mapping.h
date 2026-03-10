#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>
#include "odometry.h"

#define MAP_SIZE_X 40
#define MAP_SIZE_Y 40

#define CELL_UNKNOWN 0
#define CELL_CLEANED 1
#define CELL_OBSTACLE 2

void MAP_Init(void);

void MAP_Update(RobotPose pose);

uint8_t MAP_GetCell(int x, int y);

void MAP_SetObstacle(int x, int y);

#endif

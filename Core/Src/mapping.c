#include "mapping.h"

#define CELL_SIZE 0.25f   // meters per grid cell

static uint8_t grid[MAP_SIZE_X][MAP_SIZE_Y];

void MAP_Init(void)
{
    for(int x=0;x<MAP_SIZE_X;x++)
    {
        for(int y=0;y<MAP_SIZE_Y;y++)
        {
            grid[x][y] = CELL_UNKNOWN;
        }
    }
}

void MAP_Update(RobotPose pose)
{
    int gx = (int)(pose.x / CELL_SIZE);
    int gy = (int)(pose.y / CELL_SIZE);

    if(gx < 0 || gx >= MAP_SIZE_X)
        return;

    if(gy < 0 || gy >= MAP_SIZE_Y)
        return;

    grid[gx][gy] = CELL_CLEANED;
}

uint8_t MAP_GetCell(int x, int y)
{
    if(x < 0 || x >= MAP_SIZE_X)
        return CELL_UNKNOWN;

    if(y < 0 || y >= MAP_SIZE_Y)
        return CELL_UNKNOWN;

    return grid[x][y];
}

void MAP_SetObstacle(int x, int y)
{
    if(x < 0 || x >= MAP_SIZE_X)
        return;

    if(y < 0 || y >= MAP_SIZE_Y)
        return;

    grid[x][y] = CELL_OBSTACLE;
}

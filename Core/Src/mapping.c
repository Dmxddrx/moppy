#include "mapping.h"
#include <math.h>

static int world_to_map_x(float x)
{
    return (int)(x / MAP_RESOLUTION) + MAP_SIZE_X / 2;
}

static int world_to_map_y(float y)
{
    return (int)(y / MAP_RESOLUTION) + MAP_SIZE_Y / 2;
}

void Map_Init(Map *map)
{
    for(int i=0;i<MAP_SIZE_X;i++)
    {
        for(int j=0;j<MAP_SIZE_Y;j++)
        {
            map->grid[i][j] = CELL_UNKNOWN;
        }
    }

    map->robot_x = 0;
    map->robot_y = 0;
    map->robot_theta = 0;
}

void Map_UpdateRobotPose(Map *map, float x, float y, float theta)
{
    map->robot_x = x;
    map->robot_y = y;
    map->robot_theta = theta;
}

void Map_UpdateUltrasonic(Map *map, float distance, float sensor_angle)
{
    float global_angle = map->robot_theta + sensor_angle;

    float hit_x = map->robot_x + distance * cosf(global_angle);
    float hit_y = map->robot_y + distance * sinf(global_angle);

    int cell_x = world_to_map_x(hit_x);
    int cell_y = world_to_map_y(hit_y);

    if(cell_x >=0 && cell_x < MAP_SIZE_X &&
       cell_y >=0 && cell_y < MAP_SIZE_Y)
    {
        map->grid[cell_x][cell_y] = CELL_OCCUPIED;
    }

    int robot_cell_x = world_to_map_x(map->robot_x);
    int robot_cell_y = world_to_map_y(map->robot_y);

    if(robot_cell_x >=0 && robot_cell_x < MAP_SIZE_X &&
       robot_cell_y >=0 && robot_cell_y < MAP_SIZE_Y)
    {
        map->grid[robot_cell_x][robot_cell_y] = CELL_FREE;
    }
}

uint8_t Map_GetCell(Map *map, int x, int y)
{
    if(x <0 || x >= MAP_SIZE_X || y <0 || y >= MAP_SIZE_Y)
        return CELL_OCCUPIED;

    return map->grid[x][y];
}

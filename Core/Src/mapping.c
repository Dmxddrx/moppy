#include "mapping.h"
#include <math.h>

#define DEG_TO_RAD  0.0174533f

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
    for(int i = 0; i < MAP_SIZE_X; i++)
        for(int j = 0; j < MAP_SIZE_Y; j++)
            map->grid[i][j] = CELL_UNKNOWN;

    map->robot_x     = 0.0f;
    map->robot_y     = 0.0f;
    map->robot_theta = 0.0f;
}

void Map_UpdateRobotPose(Map *map, float x, float y, float theta)
{
    map->robot_x     = x;
    map->robot_y     = y;
    map->robot_theta = theta;

    /* Mark robot's current cell as free */
    int cx = world_to_map_x(x);
    int cy = world_to_map_y(y);

    if(cx >= 0 && cx < MAP_SIZE_X && cy >= 0 && cy < MAP_SIZE_Y)
        map->grid[cx][cy] = CELL_FREE;
}

/* ================================================================
   Map_UpdateUltrasonic
   sensor_angle_deg: mounting angle in degrees relative to robot front
   ================================================================ */
void Map_UpdateUltrasonic(Map *map, float distance, float sensor_angle_deg)
{
    if(distance <= 0.0f) return;

    float global_angle = map->robot_theta + sensor_angle_deg * DEG_TO_RAD;

    float hit_x = map->robot_x + distance * cosf(global_angle);
    float hit_y = map->robot_y + distance * sinf(global_angle);

    int cell_x = world_to_map_x(hit_x);
    int cell_y = world_to_map_y(hit_y);

    if(cell_x >= 0 && cell_x < MAP_SIZE_X &&
       cell_y >= 0 && cell_y < MAP_SIZE_Y)
        map->grid[cell_x][cell_y] = CELL_OCCUPIED;
}

/* ================================================================
   Map_UpdateIR
   IR gives no distance — marks CELL_OCCUPIED at IR_RANGE_M
   in the sensor's mounting direction when triggered.
   ================================================================ */
void Map_UpdateIR(Map *map, uint8_t sensor_index, uint8_t triggered)
{
    if(!triggered) return;

    static const float ir_angles_deg[4] = {
        IR_ANGLE_0, IR_ANGLE_1, IR_ANGLE_2, IR_ANGLE_3
    };

    if(sensor_index > 3) return;

    float global_angle = map->robot_theta
                       + ir_angles_deg[sensor_index] * DEG_TO_RAD;

    float hit_x = map->robot_x + IR_RANGE_M * cosf(global_angle);
    float hit_y = map->robot_y + IR_RANGE_M * sinf(global_angle);

    int cell_x = world_to_map_x(hit_x);
    int cell_y = world_to_map_y(hit_y);

    if(cell_x >= 0 && cell_x < MAP_SIZE_X &&
       cell_y >= 0 && cell_y < MAP_SIZE_Y)
        map->grid[cell_x][cell_y] = CELL_OCCUPIED;
}

uint8_t Map_GetCell(Map *map, int x, int y)
{
    if(x < 0 || x >= MAP_SIZE_X || y < 0 || y >= MAP_SIZE_Y)
        return CELL_OCCUPIED;

    return map->grid[x][y];
}

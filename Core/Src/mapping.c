#include "mapping.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define DEG_TO_RAD  (3.14159265f / 180.0f)

/* ─────────────────────────────────────────────────────────────── */
/*  Map_Init                                                        */
/* ─────────────────────────────────────────────────────────────── */
void Map_Init(Map *map)
{
    memset(map->grid, CELL_UNKNOWN, sizeof(map->grid));

    /* Start robot at centre of the 100×100 grid                   */
    map->robot_x     = (MAP_SIZE_X * 0.5f) * MAP_RESOLUTION;  /* 15.0 m */
    map->robot_y     = (MAP_SIZE_Y * 0.5f) * MAP_RESOLUTION;  /* 15.0 m */
    map->robot_theta = 0.0f;
    map->cells_cleaned = 0;
}

/* ─────────────────────────────────────────────────────────────── */
/*  Internal helpers                                                */
/* ─────────────────────────────────────────────────────────────── */
static inline int clamp_x(int v) {
    if (v < 0)           return 0;
    if (v >= MAP_SIZE_X) return MAP_SIZE_X - 1;
    return v;
}
static inline int clamp_y(int v) {
    if (v < 0)           return 0;
    if (v >= MAP_SIZE_Y) return MAP_SIZE_Y - 1;
    return v;
}
static inline int in_bounds(int x, int y) {
    return (x >= 0 && x < MAP_SIZE_X && y >= 0 && y < MAP_SIZE_Y);
}

/* ─────────────────────────────────────────────────────────────── */
/*  Map_UpdateRobotPose                                             */
/*  Marks the robot's current cell as CELL_FREE (cleaned).         */
/* ─────────────────────────────────────────────────────────────── */
void Map_UpdateRobotPose(Map *map, float x, float y, float theta_deg)
{
    map->robot_x     = x;
    map->robot_y     = y;
    map->robot_theta = theta_deg;

    int cx = clamp_x((int)(x / MAP_RESOLUTION));
    int cy = clamp_y((int)(y / MAP_RESOLUTION));

    /* Only mark free if the cell wasn't already an obstacle */
    if (map->grid[cx][cy] == CELL_UNKNOWN) {
        map->grid[cx][cy] = CELL_FREE;
        map->cells_cleaned++;
    } else if (map->grid[cx][cy] == CELL_FREE) {
        /* already clean — no change, no double-count */
    }
    /* CELL_OCCUPIED stays occupied (obstacle wins)                */
}

/* ─────────────────────────────────────────────────────────────── */
/*  Map_UpdateUltrasonic                                            */
/*  sensor_angle_deg: mounting angle relative to robot forward     */
/*  Marks the endpoint as CELL_OCCUPIED.                           */
/*  Clears cells along the ray (Bresenham) as seen-free space.     */
/* ─────────────────────────────────────────────────────────────── */
void Map_UpdateUltrasonic(Map *map, float distance_m, float sensor_angle_deg)
{
    if (distance_m <= 0.02f) return;               /* ignore noise          */

    /* World-frame angle: compass heading convention
       (0=North, 90=East, CW positive)
       dx uses sin, dy uses cos in NED / compass frame                        */
    float world_angle_rad = (map->robot_theta + sensor_angle_deg) * DEG_TO_RAD;
    float sin_a = sinf(world_angle_rad);
    float cos_a = cosf(world_angle_rad);

    /* Robot grid position */
    int rx = (int)(map->robot_x / MAP_RESOLUTION);
    int ry = (int)(map->robot_y / MAP_RESOLUTION);

    if (distance_m < US_MAP_MAX_M) {
        /* ── Mark obstacle endpoint ─────────────────────────── */
        float obs_x = map->robot_x + distance_m * sin_a;
        float obs_y = map->robot_y + distance_m * cos_a;
        int   ox    = (int)(obs_x / MAP_RESOLUTION);
        int   oy    = (int)(obs_y / MAP_RESOLUTION);

        if (in_bounds(ox, oy)) {
            map->grid[ox][oy] = CELL_OCCUPIED;
        }

        /* ── Bresenham ray: mark intermediate cells as seen ─── */
        int x0 = rx, y0 = ry;
        int x1 = clamp_x(ox), y1 = clamp_y(oy);
        int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
        int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
        int err = dx + dy;

        while (x0 != x1 || y0 != y1) {
            if (!in_bounds(x0, y0)) break;
            /* Don't override cleaned cells or already-known obstacles */
            if (map->grid[x0][y0] == CELL_UNKNOWN) {
                /* "seen free" but not cleaned — keep as UNKNOWN for now
                    so the floor robot still needs to physically visit it */
            }
            int e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }
    /* If distance >= US_MAP_MAX_M: nothing to mark (open space)   */
}

/* ─────────────────────────────────────────────────────────────── */
/*  Map_GetCell                                                     */
/* ─────────────────────────────────────────────────────────────── */
uint8_t Map_GetCell(const Map *map, int x, int y)
{
    if (!in_bounds(x, y)) return CELL_UNKNOWN;
    return (uint8_t)map->grid[x][y];
}

/* ─────────────────────────────────────────────────────────────── */
/*  Map_GetViewport                                                 */
/*  Calculates top-left (vx0, vy0) of the 8×4 OLED viewport,      */
/*  centred on the robot and clamped to grid bounds.               */
/* ─────────────────────────────────────────────────────────────── */
void Map_GetViewport(const Map *map, int *vx0, int *vy0)
{
    int rcx = Map_RobotCellX(map);
    int rcy = Map_RobotCellY(map);

    int vx = rcx - VIEW_COLS / 2;
    int vy = rcy - VIEW_ROWS / 2;

    /* Clamp so viewport never goes outside the grid */
    if (vx < 0)                       vx = 0;
    if (vy < 0)                       vy = 0;
    if (vx > MAP_SIZE_X - VIEW_COLS)  vx = MAP_SIZE_X - VIEW_COLS;
    if (vy > MAP_SIZE_Y - VIEW_ROWS)  vy = MAP_SIZE_Y - VIEW_ROWS;

    *vx0 = vx;
    *vy0 = vy;
}

/* ─────────────────────────────────────────────────────────────── */
int Map_RobotCellX(const Map *map) {
    return clamp_x((int)(map->robot_x / MAP_RESOLUTION));
}
int Map_RobotCellY(const Map *map) {
    return clamp_y((int)(map->robot_y / MAP_RESOLUTION));
}

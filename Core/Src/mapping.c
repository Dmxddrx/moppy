#include "mapping.h"
#include <string.h>  /* String & Memory: Gives you memset() to instantly clear your 10,000-cell map array to 0 */
#include <math.h>    /* Math: Gives you sinf(), cosf(), and atan2f() for your compass and LiDAR trigonometry */
#include <stdlib.h>  /* Standard Library: Gives you abs() to calculate absolute values (like in your Bresenham ray-casting) */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ─────────────────────────────────────────────────────────────── */
/* Internal helpers (Protects the RAM from crashes!)              */
/* ─────────────────────────────────────────────────────────────── */
static inline int clamp_x(int v) {
    if (v < 0) return 0;
    if (v >= MAP_GRID_SIZE) return MAP_GRID_SIZE - 1;
    return v;
}

static inline int clamp_y(int v) {
    if (v < 0) return 0;
    if (v >= MAP_GRID_SIZE) return MAP_GRID_SIZE - 1;
    return v;
}

static inline int in_bounds(int x, int y) {
    return (x >= 0 && x < MAP_GRID_SIZE && y >= 0 && y < MAP_GRID_SIZE);
}

/* ─────────────────────────────────────────────────────────────── */
/* Map_Init                                                       */
/* ─────────────────────────────────────────────────────────────── */
void Map_Init(Map *map)
{
    /* Set entire map to 0 (CELL_UNCLEANED) */
    memset(map->grid, CELL_UNCLEANED, sizeof(map->grid));

    /* Start robot perfectly in the center of the 15x15m world (7.5m) */
    map->robot_x     = (MAP_GRID_SIZE * 0.5f) * MAP_CELL_SIZE;
    map->robot_y     = (MAP_GRID_SIZE * 0.5f) * MAP_CELL_SIZE;
    map->robot_theta = 0.0f;
    map->cells_cleaned = 0;
}

/* ─────────────────────────────────────────────────────────────── */
/* Map_UpdateRobotPose                                            */
/* Updates the robot's memory of where it is, and increments the  */
/* cleaning counter for the floor directly underneath it!         */
/* ─────────────────────────────────────────────────────────────── */
void Map_UpdateRobotPose(Map *map, float x, float y, float theta_deg)
{
    map->robot_x     = x;
    map->robot_y     = y;
    map->robot_theta = theta_deg;

    int cx = clamp_x((int)(x / MAP_CELL_SIZE));
    int cy = clamp_y((int)(y / MAP_CELL_SIZE));

    /* Rule 1: Don't accidentally clean a wall! */
    if (map->grid[cx][cy] != CELL_OBSTACLE) {

        /* If this is the FIRST time hitting this cell, increment the global statistic */
        if (map->grid[cx][cy] == CELL_UNCLEANED) {
            map->cells_cleaned++;
        }

        /* Rule 2: Increment the times this specific cell was cleaned (max 254) */
        if (map->grid[cx][cy] < 254) {
            map->grid[cx][cy]++;
        }
    }
}

/* ─────────────────────────────────────────────────────────────── */
/* Map_MarkObstacle (For future LiDAR use)                        */
/* ─────────────────────────────────────────────────────────────── */
void Map_MarkObstacle(Map *map, float x, float y) {
    int cx = clamp_x((int)(x / MAP_CELL_SIZE));
    int cy = clamp_y((int)(y / MAP_CELL_SIZE));
    map->grid[cx][cy] = CELL_OBSTACLE;
}

/* ─────────────────────────────────────────────────────────────── */
/* Getters                                                        */
/* ─────────────────────────────────────────────────────────────── */
uint8_t Map_GetCellStatus(Map *map, float x, float y) {
    int cx = clamp_x((int)(x / MAP_CELL_SIZE));
    int cy = clamp_y((int)(y / MAP_CELL_SIZE));
    return map->grid[cx][cy];
}

uint8_t Map_GetCellByIndex(Map *map, int x_idx, int y_idx) {
    if (!in_bounds(x_idx, y_idx)) return CELL_OBSTACLE; /* Assume out-of-bounds is a wall */
    return map->grid[x_idx][y_idx];
}

/* ─────────────────────────────────────────────────────────────── */
/* Map_GetViewport (The Scrolling Minimap Engine)                 */
/* ─────────────────────────────────────────────────────────────── */
void Map_GetViewport(const Map *map, int *vx0, int *vy0)
{
    int rcx = Map_RobotCellX(map);
    int rcy = Map_RobotCellY(map);

    int vx = rcx - VIEW_COLS / 2;
    int vy = rcy - VIEW_ROWS / 2;

    /* Clamp so viewport never tries to render outside the map array */
    if (vx < 0)                           vx = 0;
    if (vy < 0)                           vy = 0;
    if (vx > MAP_GRID_SIZE - VIEW_COLS)   vx = MAP_GRID_SIZE - VIEW_COLS;
    if (vy > MAP_GRID_SIZE - VIEW_ROWS)   vy = MAP_GRID_SIZE - VIEW_ROWS;

    *vx0 = vx;
    *vy0 = vy;
}

int Map_RobotCellX(const Map *map) {
    return clamp_x((int)(map->robot_x / MAP_CELL_SIZE));
}
int Map_RobotCellY(const Map *map) {
    return clamp_y((int)(map->robot_y / MAP_CELL_SIZE));
}

/* ─────────────────────────────────────────────────────────────── */
/* Map_UpdateLiDAR (Ray-casting for 15cm Grid)                     */
/* sensor_angle_deg: mounting angle relative to robot forward      */
/* Marks the endpoint as CELL_OBSTACLE (255) and clears the path.  */
/* ─────────────────────────────────────────────────────────────── */
void Map_UpdateLiDAR(Map *map, float distance_m, float sensor_angle_deg)
{
    if (distance_m <= 0.02f) return; /* ignore noise */

    /* World-frame angle: compass heading convention (0=North, 90=East) */
    float world_angle_rad = (map->robot_theta + sensor_angle_deg) * (M_PI / 180.0f);
    float sin_a = sinf(world_angle_rad);
    float cos_a = cosf(world_angle_rad);

    /* Robot grid position */
    int rx = clamp_x((int)(map->robot_x / MAP_CELL_SIZE));
    int ry = clamp_y((int)(map->robot_y / MAP_CELL_SIZE));

    /* ── Mark obstacle endpoint ─────────────────────────── */
    float obs_x = map->robot_x + (distance_m * sin_a);
    float obs_y = map->robot_y + (distance_m * cos_a);

    int ox = clamp_x((int)(obs_x / MAP_CELL_SIZE));
    int oy = clamp_y((int)(obs_y / MAP_CELL_SIZE));

    /* Plot the solid wall! */
    map->grid[ox][oy] = CELL_OBSTACLE;

    /* ── Bresenham ray: mark intermediate cells as seen ─── */
    int x0 = rx, y0 = ry;
    int x1 = ox, y1 = oy;
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (x0 != x1 || y0 != y1) {
        if (!in_bounds(x0, y0)) break;

        /* Don't override cleaned cells or already-known obstacles */
        if (map->grid[x0][y0] == CELL_UNCLEANED) {
            /* "Seen free" but not cleaned — keep as UNCLEANED for now
               so the floor robot still knows it needs to physically visit it */
        }

        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

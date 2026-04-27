#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>

/* ── Grid dimensions ────────────────────────────────────────────
   100 × 100 cells @ 15 cm each = 15 m × 15 m real-world area  */
#define MAP_GRID_SIZE    100
#define MAP_CELL_SIZE    0.15f      /* 15cm per cell */

/* ── Cell states ────────────────────────────────────────────────
   Each cell is one byte.
   0     = Uncleaned Virgin Floor
   1-253 = Number of times the robot cleaned this exact cell
   254   = LiDAR saw it's empty, but not cleaned yet (BLINK!)
   255   = Solid Wall / Obstacle                                */
#define CELL_UNCLEANED   0
#define CELL_SEEN_FREE   254
#define CELL_OBSTACLE    255

/* ── OLED viewport  ─────────────────────────────────────────────
   If using the full 128x64 screen: 8 cols x 4 rows (16px blocks) */
#define VIEW_COLS        8
#define VIEW_ROWS        4
#define BLOCK_PX         16          /* pixels per grid cell       */

typedef struct {
    uint8_t  grid[MAP_GRID_SIZE][MAP_GRID_SIZE];
    float    robot_x;        /* metres                             */
    float    robot_y;        /* metres                             */
    float    robot_theta;    /* degrees, 0=North CW (compass)      */
    uint32_t cells_cleaned;  /* Total unique cells cleaned (max 10,000) */
} Map;

/* ── Public API ─────────────────────────────────────────────────*/
void    Map_Init(Map *map);

/* Updates location, increments clean count, tracks total cleaned */
void    Map_UpdateRobotPose(Map *map, float x, float y, float theta_deg);

/* Places an obstacle on the map (for when you reconnect LiDAR) */
void    Map_MarkObstacle(Map *map, float x, float y);

/* Returns cell status (0, 1-254, or 255) */
uint8_t Map_GetCellStatus(Map *map, float x, float y);
uint8_t Map_GetCellByIndex(Map *map, int x_idx, int y_idx);

/* Viewport & Scrolling Minimap tools */
void    Map_GetViewport(const Map *map, int *vx0, int *vy0);
int     Map_RobotCellX(const Map *map);
int     Map_RobotCellY(const Map *map);
/* Call for each LiDAR sweep that returned a valid reading */
void Map_UpdateLiDAR(Map *map, float distance_m, float sensor_angle_deg);

#endif /* MAPPING_H */

#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>

/* ── Grid dimensions ────────────────────────────────────────────
   100 × 100 cells @ 15 cm each = 15 m × 15 m real-world area  */
#define MAP_GRID_SIZE    100
#define MAP_CELL_SIZE    0.15f      /* 15cm per cell */

/* Upgrading the grid to a Probabilistic System */
#define CELL_OBSTACLE_BASE 252
#define CELL_OBSTACLE_MAX  255

/* ── Cell states ────────────────────────────────────────────────
   Each cell is one byte.
   0     = Uncleaned Virgin Floor
   1-253 = Number of times the robot cleaned this exact cell
   254   = LiDAR saw it's empty, but not cleaned yet (BLINK!)
   255   = Solid Wall / Obstacle                                */
#define CELL_UNCLEANED   0
#define CELL_SEEN_FREE   251
#define CELL_OBSTACLE    255

/* ── OLED viewport  ─────────────────────────────────────────────
   If using the full 128x64 screen: 8 cols x 4 rows (16px blocks) */
#define VIEW_COLS        8
#define VIEW_ROWS        4
#define BLOCK_PX         16          /* pixels per grid cell       */

#define MAX_BCD_CELLS 32

typedef struct {
    uint8_t id;          /* Unique cell identifier (1 to 32) */
    uint8_t x_start;     /* Grid X where this cell begins */
    uint8_t x_end;       /* Grid X where this cell ends */
    uint8_t y_min;       /* Upper boundary */
    uint8_t y_max;       /* Lower boundary */
    uint8_t is_cleaned;  /* 1 if robot has swept this area, 0 if not */
} BCD_Cell;

typedef struct {
    uint8_t  grid[MAP_GRID_SIZE][MAP_GRID_SIZE];
    float    robot_x;        /* metres                             */
    float    robot_y;        /* metres                             */
    float    robot_theta;    /* degrees, 0=North CW (compass)      */
    uint32_t cells_cleaned;  /* Total unique cells cleaned (max 10,000) */

    BCD_Cell bcd_cells[MAX_BCD_CELLS];
	uint8_t  total_bcd_cells;
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
void    Map_DecayObstacles(Map *map);

void Map_Decompose(Map *map);

#endif /* MAPPING_H */

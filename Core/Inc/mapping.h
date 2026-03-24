#ifndef MAPPING_H
#define MAPPING_H

#include <stdint.h>

/* ── Grid dimensions ────────────────────────────────────────────
   100 × 100 cells @ 30 cm each = 30 m × 30 m real-world area  */
#define MAP_SIZE_X       100
#define MAP_SIZE_Y       100
#define MAP_RESOLUTION   0.30f      /* metres per cell (30 cm)   */

/* ── Cell states ────────────────────────────────────────────────
   Each cell is one byte in int8_t grid[][]                      */
#define CELL_UNKNOWN    0           /* not yet seen               */
#define CELL_FREE       1           /* robot passed here → clean  */
#define CELL_OCCUPIED   2           /* obstacle detected          */

/* ── Ultrasonic mounting angles (degrees, robot-relative)
   0° = forward, 90° = right, clockwise convention               */
#define US_ANGLE_0       0.0f       /* front                      */
#define US_ANGLE_1      90.0f       /* right                      */
#define US_ANGLE_2     180.0f       /* rear                       */
#define US_ANGLE_3     270.0f       /* left                       */

/* Max distance used for obstacle marking (anything further ignored) */
#define US_MAP_MAX_M     3.5f

/* ── OLED viewport  ─────────────────────────────────────────────
   128 px / 16 px = 8 columns, 64 px / 16 px = 4 rows = 32 blocks
   Each block is 16 × 16 px; obstacle marker = 10 × 10 px black  */
#define VIEW_COLS        8
#define VIEW_ROWS        4
#define BLOCK_PX        16          /* pixels per grid cell       */
#define OBSTACLE_PX     10          /* obstacle square size (px)  */
#define OBSTACLE_OFF     3          /* offset inside block = (16-10)/2 */

typedef struct {
    int8_t  grid[MAP_SIZE_X][MAP_SIZE_Y];
    float   robot_x;        /* metres                             */
    float   robot_y;        /* metres                             */
    float   robot_theta;    /* degrees, 0=North CW (compass)      */
    uint32_t cells_cleaned; /* count of CELL_FREE cells           */
} Map;

/* ── Public API ─────────────────────────────────────────────────*/
void    Map_Init(Map *map);

/* Call every time pose changes */
void    Map_UpdateRobotPose(Map *map, float x, float y, float theta_deg);

/* Call for each sensor that returned a valid reading */
void    Map_UpdateUltrasonic(Map *map, float distance_m,
                              float sensor_angle_deg);

/* Returns CELL_UNKNOWN / CELL_FREE / CELL_OCCUPIED */
uint8_t Map_GetCell(const Map *map, int x, int y);

/* Returns top-left corner of 8×4 viewport centered on robot */
void    Map_GetViewport(const Map *map, int *vx0, int *vy0);

/* Robot grid position helpers */
int     Map_RobotCellX(const Map *map);
int     Map_RobotCellY(const Map *map);

#endif /* MAPPING_H */

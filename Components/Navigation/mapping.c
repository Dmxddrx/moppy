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

    /* CRITICAL FIX: Don't treat tentative or confirmed walls as clear floor! */
	if (map->grid[cx][cy] < CELL_OBSTACLE_BASE) {

		/* If this is the FIRST time hitting this cell */
		if (map->grid[cx][cy] == CELL_UNCLEANED || map->grid[cx][cy] == CELL_SEEN_FREE) {
			map->cells_cleaned++;
			map->grid[cx][cy] = 1;
		}
		/* Max cleaning passes capped safely below obstacle values */
		else if (map->grid[cx][cy] < (CELL_OBSTACLE_BASE - 1)) {
			map->grid[cx][cy]++;
		}
	}
}

void Map_DecayObstacles(Map *map) {
    for (int x = 0; x < MAP_GRID_SIZE; x++) {
        for (int y = 0; y < MAP_GRID_SIZE; y++) {
            if (map->grid[x][y] >= CELL_OBSTACLE_BASE) {
                map->grid[x][y]--;
                if (map->grid[x][y] < CELL_OBSTACLE_BASE) {
                    map->grid[x][y] = CELL_SEEN_FREE;
                }
            }
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

    /* ── Mark obstacle endpoint (Increase Confidence) ── */
	if (map->grid[ox][oy] < CELL_OBSTACLE_BASE) {
		map->grid[ox][oy] = CELL_OBSTACLE_BASE;
	} else if (map->grid[ox][oy] < CELL_OBSTACLE_MAX) {
		map->grid[ox][oy]++;
	}

	/* ── Bresenham ray: mark intermediate cells as seen (Decrease Confidence) ── */
	int x0 = rx, y0 = ry;
	int x1 = ox, y1 = oy;
	int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
	int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
	int err = dx + dy;

	while (x0 != x1 || y0 != y1) {
		if (!in_bounds(x0, y0)) break;

		/* If passing through a "wall", lower the confidence of the wall */
		if (map->grid[x0][y0] >= CELL_OBSTACLE_BASE) {
			map->grid[x0][y0]--;
			/* If the confidence drops too low, it's open space again */
			if (map->grid[x0][y0] < CELL_OBSTACLE_BASE) {
				map->grid[x0][y0] = CELL_SEEN_FREE;
			}
		}
		/* Otherwise, simply mark unexplored areas as seen */
		else if (map->grid[x0][y0] == CELL_UNCLEANED) {
			map->grid[x0][y0] = CELL_SEEN_FREE;
		}

		int e2 = 2 * err;
		if (e2 >= dy) { err += dy; x0 += sx; }
		if (e2 <= dx) { err += dx; y0 += sy; }
	}
}


void Map_Decompose(Map *map)
{
    /* Reset current cell data */
    map->total_bcd_cells = 0;
    memset(map->bcd_cells, 0, sizeof(map->bcd_cells));

    int current_cell_idx = 0;
    int in_open_space = 0;
    int y_start = 0;

    /* Phase 1: The Sweep-Line */
    /* We sweep left to right across the 15x15m grid */
    for (int x = 0; x < MAP_GRID_SIZE; x++) {

		in_open_space = 0;

		for (int y = 0; y < MAP_GRID_SIZE; y++) {

			/* Is this space clear of walls? */
			int is_free = (map->grid[x][y] < CELL_OBSTACLE_BASE);

			/* Transition: Wall -> Open Space (Start a new block) */
			if (is_free && !in_open_space) {
				in_open_space = 1;
				y_start = y;
			}
			/* Transition: Open Space -> Wall (End the block) */
			else if (!is_free && in_open_space) {
				in_open_space = 0;

				if (current_cell_idx >= MAX_BCD_CELLS) break;

				/* Register the cell block */
				map->bcd_cells[current_cell_idx].id = current_cell_idx + 1;
				map->bcd_cells[current_cell_idx].x_start = x;
				map->bcd_cells[current_cell_idx].x_end = x;
				map->bcd_cells[current_cell_idx].y_min = y_start;
				map->bcd_cells[current_cell_idx].y_max = y - 1;
				map->bcd_cells[current_cell_idx].is_cleaned = 0;

				current_cell_idx++;
			}
		}

		/* THE FIX: If we reached the edge of the map while still in open space, close the block! */
		if (in_open_space) {
			if (current_cell_idx >= MAX_BCD_CELLS) break;

			map->bcd_cells[current_cell_idx].id = current_cell_idx + 1;
			map->bcd_cells[current_cell_idx].x_start = x;
			map->bcd_cells[current_cell_idx].x_end = x;
			map->bcd_cells[current_cell_idx].y_min = y_start;
			map->bcd_cells[current_cell_idx].y_max = MAP_GRID_SIZE - 1; /* Caps at 99 */
			map->bcd_cells[current_cell_idx].is_cleaned = 0;

			current_cell_idx++;
		}

		if (current_cell_idx >= MAX_BCD_CELLS) break;
	}

    map->total_bcd_cells = current_cell_idx;

    /* Phase 2: Cell Merging (To be implemented)
       In the Python code, 'get_adjacency_matrix' merges these 1-pixel wide
       blocks into massive room rectangles. For the STM32, we will loop through
       map->bcd_cells and combine any cells that share the same y_min and y_max
       and have adjacent x coordinates.
    */
    /* ... (Place this right below map->total_bcd_cells = current_cell_idx;
                inside your Map_Decompose function in mapping.c) ... */

	/* Phase 2: Cell Merging (The Adjacency Matrix alternative for STM32) */
	for (int i = 0; i < map->total_bcd_cells; i++) {
		if (map->bcd_cells[i].id == 0) continue; /* Skip cells that were already merged and deleted */

		for (int j = i + 1; j < map->total_bcd_cells; j++) {
			if (map->bcd_cells[j].id == 0) continue;

			/* Check 1: Are these two strips horizontally touching? */
			if (map->bcd_cells[i].x_end + 1 == map->bcd_cells[j].x_start) {

				/* Check 2: Do they represent the same open space?
				   (Allowing a 1-cell / 15cm tolerance for crooked walls) */
				if (abs(map->bcd_cells[i].y_min - map->bcd_cells[j].y_min) <= 1 &&
					abs(map->bcd_cells[i].y_max - map->bcd_cells[j].y_max) <= 1) {

					/* MERGE THEM! Stretch Cell A to encompass Cell B */
					map->bcd_cells[i].x_end = map->bcd_cells[j].x_end;

					/* Average the top and bottom boundaries for a smooth rectangle */
					map->bcd_cells[i].y_min = (map->bcd_cells[i].y_min + map->bcd_cells[j].y_min) / 2;
					map->bcd_cells[i].y_max = (map->bcd_cells[i].y_max + map->bcd_cells[j].y_max) / 2;

					/* Mark Cell B as deleted so it is ignored */
					map->bcd_cells[j].id = 0;
				}
			}
		}
	}

	/* Phase 3: Array Compression (Clean up the empty deleted slots) */
	int final_cell_count = 0;
	for (int i = 0; i < map->total_bcd_cells; i++) {
		if (map->bcd_cells[i].id != 0) {
			/* Shift the valid cell down to close the gap */
			map->bcd_cells[final_cell_count] = map->bcd_cells[i];

			/* Re-number the ID so they are perfectly sequential (1, 2, 3...) */
			map->bcd_cells[final_cell_count].id = final_cell_count + 1;
			final_cell_count++;
		}
	}
	map->total_bcd_cells = final_cell_count;
}

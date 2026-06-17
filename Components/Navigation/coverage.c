#include "coverage.h"
#include "mapping.h"
#include <math.h>

extern Map g_map;

/* ── Reactive Navigation State Machine ────────────────────── */
typedef enum {
    NAV_FORWARD,
    NAV_REVERSE,
	NAV_TRAPPED,
	NAV_PAUSE,
	NAV_TURN_90_PHASE1,  /* First 90-degree turn of the U-Turn */
	NAV_SHIFT_LANE,      /* Driving straight to shift to the next lane */
	NAV_TURN_90_PHASE2,   /* Second 90-degree turn to complete the U-Turn */
	NAV_WALL_FOLLOW,     /* NEW: Temporary detour around furniture */
	NAV_ESCAPE_CORNER,    /* NEW: Your old trapped logic! */
	NAV_TURN_LEFT_90,    /* Re-added from your old code */
	NAV_TURN_RIGHT_90,   /* Re-added from your old code */
	NAV_TURN_LEFT_180,
	/* ── PHASE 3: PATHFINDING STATES ── */
	NAV_TRANSIT_CALC,    /* Calculate heading to the next BCD cell */
	NAV_TRANSIT_TURN,    /* Pivot to face the next cell */
	NAV_TRANSIT_DRIVE    /* Drive to the next cell */
} NavState;

static NavState s_nav_state = NAV_FORWARD;
static NavState s_next_state = NAV_FORWARD;
static float s_target_heading = 0.0f;
static uint16_t s_reverse_timer = 0;
static uint16_t s_pause_timer = 0;
static uint16_t s_shift_timer = 0;

/* 1 = Shifting Right across the room, -1 = Shifting Left */
static int s_sweep_dir = 1;
/* NEW BCD TRACKERS */
static uint8_t s_active_cell_idx = 0; /* Tracks which BCD rectangle we are cleaning (0 to 31) */
static int s_cell_complete = 0;       /* Flag that trips to 1 when a rectangle is fully painted */
static float s_transit_target_x = 0.0f;
static float s_transit_target_y = 0.0f;

/* ═══════════════════════════════════════════════════════════════ */
/* HELPER MATH                                                     */
/* ═══════════════════════════════════════════════════════════════ */
static float angle_diff(float target, float current) {
    float d = target - current;
    while(d >  180.0f) d -= 360.0f;
    while(d < -180.0f) d += 360.0f;
    return d;
}

/* ═══════════════════════════════════════════════════════════════ */
/* PUBLIC FUNCTIONS                                                */
/* ═══════════════════════════════════════════════════════════════ */
void COVERAGE_Init(float start_heading) {
    s_nav_state = NAV_FORWARD;
    s_target_heading = start_heading;
    s_sweep_dir = 1; /* Start by shifting right on the first wall */
}

CoverageCmd COVERAGE_Update(float current_yaw, int obs_F, int obs_R, int obs_L, int obs_B, int bump_detected, int is_stuck, float* out_target_yaw) {
    CoverageCmd cmd = CMD_DRIVE_FORWARD;

    /* ═══════════════════════════════════════════════════════════════ */
	/* NEW: VIRTUAL WALL LOGIC (BCD)                                   */
	/* ═══════════════════════════════════════════════════════════════ */
	int virtual_wall_hit = 0;

	/* Only run this if Map_Decompose has generated cells, and we aren't done yet */
	if (g_map.total_bcd_cells > 0 && !s_cell_complete && s_nav_state != NAV_TRANSIT_DRIVE) {
		BCD_Cell *active_cell = &g_map.bcd_cells[s_active_cell_idx];

		/* Get current grid coordinates */
		int rx = Map_RobotCellX(&g_map);
		int ry = Map_RobotCellY(&g_map);

		/* Heading Check: Driving North (+Y) */
		if (fabsf(angle_diff(0.0f, current_yaw)) < 45.0f && ry >= active_cell->y_max) {
			virtual_wall_hit = 1;
		}
		/* Heading Check: Driving South (-Y) */
		else if (fabsf(angle_diff(180.0f, current_yaw)) < 45.0f && ry <= active_cell->y_min) {
			virtual_wall_hit = 1;
		}
		/* Heading Check: Driving East (+X) */
		else if (fabsf(angle_diff(90.0f, current_yaw)) < 45.0f && rx >= active_cell->x_end) {
			virtual_wall_hit = 1;
		}
		/* Heading Check: Driving West (-X) */
		else if (fabsf(angle_diff(270.0f, current_yaw)) < 45.0f && rx <= active_cell->x_start) {
			virtual_wall_hit = 1;
		}
	}

	/* ═══════════════════════════════════════════════════════════════ */
	/* THE MASTER STATE MACHINE                                        */
	/* ═══════════════════════════════════════════════════════════════ */
    switch (s_nav_state) {

		case NAV_FORWARD:
			s_reverse_timer = 0;
			cmd = CMD_DRIVE_FORWARD;
			*out_target_yaw = s_target_heading;

			/* Priority 1: Hardware Stuck Failsafe */
			if (is_stuck) {
				s_nav_state = NAV_TRAPPED;
			}
			/* Priority 2: THE OLD TRAPPED LOGIC - Cornered by furniture! */
			else if (obs_F && obs_L && obs_R) {
				s_next_state = NAV_ESCAPE_CORNER; // Escape the cluster of chair legs
				s_nav_state = NAV_REVERSE;
				s_pause_timer = 0;
			}
			/* Priority 3: A single obstacle in the way (like a trash can or table leg) */
			else if (obs_F && !obs_L && !obs_R) {
				/* Don't ruin the lawnmower pattern! Just curve around it. */
				s_nav_state = NAV_WALL_FOLLOW;
			}
			/* Priority 4: Reached the actual main wall of the room */
			else if (bump_detected || (obs_F && (obs_L || obs_R)) || virtual_wall_hit) {
				/* Now we trigger the Boustrophedon U-Turn! */
				s_next_state = NAV_TURN_90_PHASE1;

				if (s_sweep_dir == 1) {
					s_target_heading = fmodf(current_yaw + 360.0f - 90.0f, 360.0f);
				} else {
					s_target_heading = fmodf(current_yaw + 90.0f, 360.0f);
				}

				s_nav_state = NAV_REVERSE;
				s_pause_timer = 0;
			}
			break;

		case NAV_REVERSE:
			cmd = CMD_REVERSE;
			*out_target_yaw = s_target_heading;

			s_reverse_timer++;
			if (s_reverse_timer > 100) { /* 1.0s reverse */
				s_reverse_timer = 0;

				/* SCENARIO A: Just a normal Boustrophedon U-Turn at a wall */
				if (s_next_state == NAV_TURN_90_PHASE1 || s_next_state == NAV_FORWARD) {
				    s_nav_state = NAV_PAUSE;
				    s_pause_timer = 0;
				}
				/* SCENARIO B: We are trapped in furniture! Use the smart escape! */
				else if (s_next_state == NAV_ESCAPE_CORNER) {
					if (!obs_L) {
						s_next_state = NAV_TURN_LEFT_90;
						s_target_heading = fmodf(current_yaw + 90.0f, 360.0f);
					}
					else if (!obs_R) {
						s_next_state = NAV_TURN_RIGHT_90;
						s_target_heading = fmodf(current_yaw + 360.0f - 90.0f, 360.0f);
					} else {
						/* Both sides blocked, turn completely around! */
						s_next_state = NAV_TURN_LEFT_180;
						s_target_heading = fmodf(current_yaw + 180.0f, 360.0f);
					}
					s_nav_state = NAV_PAUSE;
					s_pause_timer = 0;
				}
			}
			break;

		case NAV_TURN_90_PHASE1:
			cmd = (s_sweep_dir == 1) ? CMD_TURN_RIGHT : CMD_TURN_LEFT;
			*out_target_yaw = s_target_heading;

			if (fabsf(angle_diff(s_target_heading, current_yaw)) <= 3.0f) {
				/* First turn complete. Now drive forward to create the lane width */
				s_nav_state = NAV_PAUSE;
				s_next_state = NAV_SHIFT_LANE;
				s_pause_timer = 0;
				s_shift_timer = 0;
			}
			break;

		case NAV_SHIFT_LANE:
			cmd = CMD_DRIVE_FORWARD;
			*out_target_yaw = s_target_heading;
			s_shift_timer++;

			/* EDGE CASE: Hit a side wall or finished the BCD Cell */
			if (obs_F || bump_detected || virtual_wall_hit) {

				/* If we hit a VIRTUAL wall on the side, the rectangle is completely painted! */
				if (virtual_wall_hit) {
					s_cell_complete = 1;         /* Mark this cell as done */
					s_nav_state = NAV_PAUSE;     /* Stop the motors */
					s_next_state = NAV_PAUSE;    /* Await further instructions (Pathfinding Phase) */
					s_pause_timer = 0;
				}
				/* Otherwise, it was a physical trap in a room corner (Old Logic) */
				else {
					s_sweep_dir = -s_sweep_dir;
					s_target_heading = fmodf(current_yaw + 180.0f, 360.0f);
					s_next_state = NAV_FORWARD;
					s_nav_state = NAV_REVERSE;
				}
			}
			/* Normal Lane Shift: Drive parallel for 1.5 seconds */
			else if (s_shift_timer > 150) {
				s_nav_state = NAV_PAUSE;
				s_next_state = NAV_TURN_90_PHASE2;
				s_pause_timer = 0;

				/* Phase 2 Turn: Another 90 degrees in the same direction */
				if (s_sweep_dir == 1) {
					s_target_heading = fmodf(current_yaw + 360.0f - 90.0f, 360.0f); /* Right */
				} else {
					s_target_heading = fmodf(current_yaw + 90.0f, 360.0f); /* Left */
				}
			}
			break;

		case NAV_TURN_90_PHASE2:
			cmd = (s_sweep_dir == 1) ? CMD_TURN_RIGHT : CMD_TURN_LEFT;
			*out_target_yaw = s_target_heading;

			if (fabsf(angle_diff(s_target_heading, current_yaw)) <= 3.0f) {
				/* U-Turn complete!
				   Invert the sweep direction so the NEXT wall triggers an opposite U-Turn */
				s_sweep_dir = -s_sweep_dir;
				s_nav_state = NAV_PAUSE;
				s_next_state = NAV_FORWARD;
				s_pause_timer = 0;
			}
			break;

		case NAV_PAUSE:
			cmd = CMD_STOP;
			*out_target_yaw = s_target_heading;

			s_pause_timer++;
			if (s_pause_timer > 30) {

				/* PHASE 3 TRIGGER: If we paused because a cell finished, pathfind to the next one! */
				if (s_cell_complete && s_next_state == NAV_PAUSE) {

					if (s_active_cell_idx + 1 < g_map.total_bcd_cells) {
						s_active_cell_idx++;

						/* Calculate real-world coordinates for the center of the target cell */
						s_transit_target_x = (g_map.bcd_cells[s_active_cell_idx].x_start * MAP_CELL_SIZE) + (MAP_CELL_SIZE / 2.0f);
						s_transit_target_y = (g_map.bcd_cells[s_active_cell_idx].y_min * MAP_CELL_SIZE) + (MAP_CELL_SIZE / 2.0f);

						s_nav_state = NAV_TRANSIT_CALC;
					} else {
						/* THE ENTIRE ROOM IS CLEAN! */
						cmd = CMD_STOP;
						s_nav_state = NAV_PAUSE;
					}
				} else {
					/* Normal pause resume */
					s_nav_state = s_next_state;
				}
			}
			break;

		/* ═══════════════════════════════════════════════════════════════ */
		/* PHASE 3: POINT-TO-POINT TRANSIT LOGIC                           */
		/* ═══════════════════════════════════════════════════════════════ */
		case NAV_TRANSIT_CALC:
			cmd = CMD_STOP;

			float dx = s_transit_target_x - g_map.robot_x;
			float dy = s_transit_target_y - g_map.robot_y;

			/* Compass 0 is +Y (North), 90 is +X (East). atan2 computes this perfectly */
			float target_angle = atan2f(dx, dy) * (180.0f / M_PI);
			if (target_angle < 0.0f) target_angle += 360.0f;

			s_target_heading = target_angle;
			*out_target_yaw = s_target_heading;

			s_nav_state = NAV_TRANSIT_TURN;
			break;

		case NAV_TRANSIT_TURN:
			/* Pivot dynamically to face the calculated transit angle */
			float diff = angle_diff(s_target_heading, current_yaw);
			if (diff > 0.0f) cmd = CMD_TURN_RIGHT;
			else cmd = CMD_TURN_LEFT;

			*out_target_yaw = s_target_heading;

			if (fabsf(diff) <= 3.0f) {
				s_nav_state = NAV_TRANSIT_DRIVE;
			}
			break;

		case NAV_TRANSIT_DRIVE:
			cmd = CMD_DRIVE_FORWARD;
			*out_target_yaw = s_target_heading;

			float dist_x = s_transit_target_x - g_map.robot_x;
			float dist_y = s_transit_target_y - g_map.robot_y;
			float distance_to_target = sqrtf((dist_x * dist_x) + (dist_y * dist_y));

			/* Check if we arrived at the new cell boundary (within 15cm) */
			if (distance_to_target <= MAP_CELL_SIZE) {
				s_cell_complete = 0;           /* Target reached! Switch back to cleaning mode */
				s_nav_state = NAV_FORWARD;
				s_sweep_dir = 1;               /* Reset sweep to start fresh */
			}
			/* Obstacle Avoidance during transit */
			else if (obs_F || bump_detected) {
				s_nav_state = NAV_WALL_FOLLOW; /* Detour around the obstacle */
			}
			break;
		/* ═══════════════════════════════════════════════════════════════ */

        case NAV_TRAPPED:
			/* Halt all motors and wait for a human or object to move out of the way */
			cmd = CMD_STOP;
			*out_target_yaw = s_target_heading;

			if (!obs_F || !obs_L || !obs_R || !obs_B) {
				/* An opening appeared! Go back to Forward mode to recalculate the route */
				s_nav_state = NAV_FORWARD;
			}
			break;

        case NAV_WALL_FOLLOW:
			/* Temporarily bypass the Boustrophedon to curve around a single obstacle */
			cmd = CMD_TURN_RIGHT; /* (Replace with actual WALLFOLLOW_Update math later) */
			*out_target_yaw = s_target_heading;

			if (!obs_F) {
				if (s_cell_complete) {
					/* If we were in transit when we dodged, recalculate the path! */
					s_nav_state = NAV_TRANSIT_CALC;
				} else {
					/* If we were sweeping, resume the Boustrophedon pattern */
					s_nav_state = NAV_FORWARD;
				}
			}
			break;

        case NAV_TURN_LEFT_180:
        case NAV_TURN_RIGHT_90:
        case NAV_TURN_LEFT_90:
            /* Output the correct turning command to the hardware driver */
            if (s_nav_state == NAV_TURN_LEFT_90 || s_nav_state == NAV_TURN_LEFT_180) {
                cmd = CMD_TURN_LEFT;
            } else {
                cmd = CMD_TURN_RIGHT;
            }

            *out_target_yaw = s_target_heading;

            /* If the compass is within 3 degrees of target, lock the FSM back to Forward! */
            if (fabsf(angle_diff(s_target_heading, current_yaw)) <= 3.0f) {
            	s_next_state = NAV_FORWARD;
                s_nav_state = NAV_PAUSE;
				s_pause_timer = 0;
            }
            break;
        case NAV_ESCAPE_CORNER:
			/* Satisfies compiler - this state is handled inside NAV_REVERSE as a flag */
			break;
    }

    return cmd;
}

/* ═══════════════════════════════════════════════════════════════ */
/* TELEMETRY GETTERS                                               */
/* ═══════════════════════════════════════════════════════════════ */
int COVERAGE_GetNavState(void) {
    return (int)s_nav_state;
}

int COVERAGE_GetActiveBCDCell(void) {
    return (int)s_active_cell_idx;
}

float COVERAGE_GetTargetHeading(void) {
    return s_target_heading;
}

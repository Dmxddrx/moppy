#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

void WALLFOLLOW_Init(void);

void WALLFOLLOW_Update(float current_distance, float desired_distance,
                       float current_heading,  float wall_parallel_heading,
                       int base_speed,
                       int *left, int *right);

#endif

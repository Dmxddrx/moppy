#ifndef WALL_FOLLOW_H
#define WALL_FOLLOW_H

void WALLFOLLOW_Init(void);

void WALLFOLLOW_Update(float distance,
                       float desired,
                       int base_speed,
                       int *left,
                       int *right);

#endif

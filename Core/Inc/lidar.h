#ifndef LIDAR_H
#define LIDAR_H

#include "main.h"

/* * The default VL53L0X 7-bit address is 0x29.
 * We are assigning these new 7-bit addresses to the 4 sensors.
 */
#define LIDAR_ADDR_0 0x30  /* Front */
#define LIDAR_ADDR_1 0x31  /* Right */
#define LIDAR_ADDR_2 0x32  /* Back  */
#define LIDAR_ADDR_3 0x33  /* Left  */

void  LIDAR_Init(void);
float LIDAR_GetDistance(uint8_t index);

#endif /* LIDAR_H */

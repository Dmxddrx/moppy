#ifndef OLED_GUI_H
#define OLED_GUI_H

#include "general.h" /* To bring in Map, CoverageCmd, and WIFI_BRIDGE macros */

/* Expose the render functions to general.c */
void render_dashboard_page(void);
void render_compass_page(void);
void render_map_page(void);
void render_wifi_page(void);
void render_calib_page(void);

#endif /* OLED_GUI_H */

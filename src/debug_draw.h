#ifndef DEBUG_DRAW_H
#define DEBUG_DRAW_H

#include "raylib.h"

void debug_draw_update(void);
void debug_draw_ray(Ray ray, Color color, float duration);
void debug_draw_line(Vector3 start, Vector3 end, Color color, float duration);

#endif // DEBUG_DRAW_H
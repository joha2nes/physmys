#include "debug_draw.h"
#include "list.h"
#include "raymath.h"

typedef struct {
    Ray ray;
    Color color;
    float duration;
} Ray_Item;

typedef struct {
    Vector3 start;
    Vector3 end;
    Color color;
    float duration;
} Line_Item;

struct {
    list(Ray_Item) rays;
    list(Line_Item) lines;
} debug_state = {0};

void debug_draw_update(void) {
    for (int i = 0; i < debug_state.rays.count; i++) {
        Ray_Item item = debug_state.rays.items[i];
        Vector3 end = Vector3Add(item.ray.position, item.ray.direction);
        DrawLine3D(item.ray.position, end, item.color);
    }
    for (int i = 0; i < debug_state.lines.count; i++) {
        Line_Item item = debug_state.lines.items[i];
        DrawLine3D(item.start, item.end, item.color);
    }

    float dt = GetFrameTime();

    for (int i = debug_state.rays.count - 1; i >= 0; i--) {
        debug_state.rays.items[i].duration -= dt;
        if (debug_state.rays.items[i].duration < 0) {
            list_remove_at(&debug_state.rays, i);
        }
    }
    for (int i = debug_state.lines.count - 1; i >= 0; i--) {
        debug_state.lines.items[i].duration -= dt;
        if (debug_state.lines.items[i].duration < 0) {
            list_remove_at(&debug_state.lines, i);
        }
    }
}

void debug_draw_ray(Ray ray, Color color, float duration) {
    Ray_Item item = {
        .ray = ray,
        .color = color,
        .duration = duration
    };
    list_add(&debug_state.rays, item);
}

void debug_draw_line(Vector3 start, Vector3 end, Color color, float duration) {
    Line_Item item = {
        .start = start,
        .end = end,
        .color = color,
        .duration = duration
    };
    list_add(&debug_state.lines, item);
}

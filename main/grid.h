#pragma once
#include <stdbool.h>
#include <stdint.h>

#define GRID_SIDE 21
#define GRID_CENTER (GRID_SIDE / 2)
#define GRID_COUNT (GRID_SIDE * GRID_SIDE)

typedef enum { CELL_UNKNOWN, CELL_FREE, CELL_BLOCKED } cell_t;
typedef enum { NORTH, EAST, SOUTH, WEST } heading_t;
typedef struct { int x, y; } cell_pos_t;
typedef struct {
    uint8_t cells[GRID_COUNT];
    uint8_t scans[GRID_COUNT];
} grid_t;

bool grid_contains(cell_pos_t cell);
bool grid_equal(cell_pos_t a, cell_pos_t b);
cell_pos_t grid_offset(cell_pos_t from, heading_t heading);
heading_t grid_bearing(cell_pos_t from, cell_pos_t toward);
cell_t grid_get(const grid_t *grid, cell_pos_t cell);
void grid_set(grid_t *grid, cell_pos_t cell, cell_t state);
uint8_t grid_scans(const grid_t *grid, cell_pos_t cell);
void grid_mark_scanned(grid_t *grid, cell_pos_t cell, heading_t heading);
void grid_observe(grid_t *grid, cell_pos_t start, heading_t heading, int cm, int cell_cm,
                  int clearance_cm);
bool grid_next_step(const grid_t *grid, cell_pos_t start, const cell_pos_t *goal,
                    cell_pos_t *step, cell_pos_t *target);

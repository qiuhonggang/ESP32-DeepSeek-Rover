#include "grid.h"
#include <stddef.h>

static int index_of(cell_pos_t cell) { return cell.y * GRID_SIDE + cell.x; }
static cell_pos_t from_index(int i) { return (cell_pos_t){i % GRID_SIDE, i / GRID_SIDE}; }

bool grid_contains(cell_pos_t cell)
{
    return cell.x >= 0 && cell.x < GRID_SIDE && cell.y >= 0 && cell.y < GRID_SIDE;
}

bool grid_equal(cell_pos_t a, cell_pos_t b) { return a.x == b.x && a.y == b.y; }

cell_pos_t grid_offset(cell_pos_t from, heading_t heading)
{
    static const int dx[] = {0, 1, 0, -1};
    static const int dy[] = {-1, 0, 1, 0};
    return (cell_pos_t){from.x + dx[heading], from.y + dy[heading]};
}

heading_t grid_bearing(cell_pos_t from, cell_pos_t toward)
{
    if (toward.x > from.x) return EAST;
    if (toward.x < from.x) return WEST;
    return toward.y > from.y ? SOUTH : NORTH;
}

cell_t grid_get(const grid_t *grid, cell_pos_t cell)
{
    return grid_contains(cell) ? (cell_t)grid->cells[index_of(cell)] : CELL_BLOCKED;
}

void grid_set(grid_t *grid, cell_pos_t cell, cell_t state)
{
    if (grid_contains(cell)) grid->cells[index_of(cell)] = state;
}

uint8_t grid_scans(const grid_t *grid, cell_pos_t cell)
{
    return grid_contains(cell) ? grid->scans[index_of(cell)] : 15;
}

void grid_mark_scanned(grid_t *grid, cell_pos_t cell, heading_t heading)
{
    if (grid_contains(cell)) grid->scans[index_of(cell)] |= (uint8_t)(1u << heading);
}

void grid_observe(grid_t *grid, cell_pos_t start, heading_t heading, int cm, int cell_cm,
                  int clearance_cm)
{
    if (cm <= 0 || cell_cm <= 0) return;
    cell_pos_t cell = start;
    for (int i = 1; i < GRID_SIDE; ++i) {
        cell = grid_offset(cell, heading);
        if (!grid_contains(cell)) break;
        if (i * cell_cm + clearance_cm <= cm) {
            grid_set(grid, cell, CELL_FREE);
        } else {
            // Treat any cell without traversal clearance as blocked along this ray.
            grid_set(grid, cell, CELL_BLOCKED);
            break;
        }
    }
}

// BFS is shortest-path planning on this uniform-cost 4-neighbor grid.
// Without a goal it selects the nearest reachable cell with an unfinished scan.
bool grid_next_step(const grid_t *grid, cell_pos_t start, const cell_pos_t *goal,
                    cell_pos_t *step, cell_pos_t *target)
{
    if (grid_get(grid, start) != CELL_FREE || (goal && grid_get(grid, *goal) != CELL_FREE)) return false;
    int16_t parents[GRID_COUNT], queue[GRID_COUNT];
    for (int i = 0; i < GRID_COUNT; ++i) parents[i] = -1;
    int root = index_of(start);
    parents[root] = (int16_t)root;
    queue[0] = (int16_t)root;
    int found = -1, read = 0, write = 1;
    while (read < write) {
        int current = queue[read++];
        cell_pos_t cell = from_index(current);
        if (goal ? grid_equal(cell, *goal) : grid_scans(grid, cell) != 15) {
            found = current;
            break;
        }
        for (heading_t heading = NORTH; heading <= WEST; ++heading) {
            cell_pos_t adjacent = grid_offset(cell, heading);
            if (grid_get(grid, adjacent) != CELL_FREE) continue;
            int next = index_of(adjacent);
            if (parents[next] >= 0) continue;
            parents[next] = (int16_t)current;
            queue[write++] = (int16_t)next;
        }
    }
    if (found < 0) return false;
    if (target) *target = from_index(found);
    while (found != root && parents[found] != root) found = parents[found];
    if (step) *step = from_index(found);
    return true;
}

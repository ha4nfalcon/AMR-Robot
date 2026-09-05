"""Warehouse layout: bigger grid with shelves, aisles, a narrow choke point,
fixed machinery/pallet obstacles, and a charging dock."""
from dataclasses import dataclass

FREE = 0
WALL = 1

# Fixed real-world obstacles (machinery block + pallet stack in an aisle)
FIXED_OBSTACLES = [
    (21, 15), (22, 15), (21, 16), (22, 16),  # parked machinery (2x2)
    (11, 11), (12, 11),                      # pallet stack in cross-aisle
]

@dataclass
class WarehouseMap:
    width: int = 28
    height: int = 18
    grid: list = None
    choke_cells: list = None  # narrow intersection cells
    pickup_points: list = None
    dropoff_points: list = None
    dock: tuple = (26, 16)  # single charging dock

    def is_free(self, x, y, blocked=None):
        if not (0 <= x < self.width and 0 <= y < self.height):
            return False
        if self.grid[y][x] == WALL:
            return False
        if blocked and (x, y) in blocked:
            return False
        return True


def build_default_map() -> WarehouseMap:
    W, H = 28, 18
    grid = [[FREE for _ in range(W)] for _ in range(H)]
    # outer walls
    for x in range(W):
        grid[0][x] = WALL
        grid[H-1][x] = WALL
    for y in range(H):
        grid[y][0] = WALL
        grid[y][W-1] = WALL

    # shelf blocks: 4 horizontal shelf rows with vertical aisles
    # shelves occupy rows (3,4),(6,7),(9,10),(12,13); gaps at x=5,9,10,14,19,24
    shelf_rows = [(3, 4), (6, 7), (9, 10), (12, 13)]
    for r1, r2 in shelf_rows:
        for x in range(2, 26):
            # leave a 1-cell wide choke corridor at x=9..10 center (narrow intersection)
            if x in (9, 10):
                continue
            # leave vertical aisles
            if x in (5, 14, 19, 24):
                continue
            grid[r1][x] = WALL
            grid[r2][x] = WALL

    # narrow intersection: force single-lane bridge at center (y=5..8, x=9)
    # block x=10 at rows 5 and 8 to create 1-wide choke
    grid[5][10] = WALL
    grid[8][10] = WALL

    # fixed real-world obstacles (machinery, pallet stacks)
    for (x, y) in FIXED_OBSTACLES:
        grid[y][x] = WALL

    choke_cells = [(9, 5), (9, 6), (9, 7), (9, 8)]

    pickup_points = [(2, 2), (25, 2), (2, 15), (25, 15), (9, 1)]
    dropoff_points = [(9, 16), (2, 5), (25, 8)]

    # ensure pickups/dropoffs are free
    for (x, y) in pickup_points + dropoff_points:
        grid[y][x] = FREE

    dock = (26, 16)
    grid[dock[1]][dock[0]] = FREE  # charging dock always clear

    return WarehouseMap(width=W, height=H, grid=grid,
                         choke_cells=choke_cells,
                         pickup_points=pickup_points,
                         dropoff_points=dropoff_points,
                         dock=dock)

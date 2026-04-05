import os
import random
from collections import deque


WIDTH = 20
HEIGHT = 20
SEED = 42

OPEN_COUNT = 5
BOTTLENECK_COUNT = 5
MAZE_COUNT = 5


def empty_grid(width, height, fill="."):
    return [[fill for _ in range(width)] for _ in range(height)]


def add_border_walls(grid):
    height = len(grid)
    width = len(grid[0])

    for x in range(width):
        grid[0][x] = "#"
        grid[height - 1][x] = "#"

    for y in range(height):
        grid[y][0] = "#"
        grid[y][width - 1] = "#"


def save_grid(grid, filename):
    with open(filename, "w", encoding="utf-8") as f:
        for row in grid:
            f.write("".join(row) + "\n")


def place_start_goal(grid, start, goal):
    sx, sy = start
    gx, gy = goal
    grid[sy][sx] = "S"
    grid[gy][gx] = "G"


def find_symbol(grid, symbol):
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell == symbol:
                return (x, y)
    return None


def is_solvable(grid):
    start = find_symbol(grid, "S")
    goal = find_symbol(grid, "G")
    if start is None or goal is None:
        return False

    height = len(grid)
    width = len(grid[0])

    q = deque([start])
    visited = {start}
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while q:
        x, y = q.popleft()
        if (x, y) == goal:
            return True

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                if (nx, ny) not in visited and grid[ny][nx] != "#":
                    visited.add((nx, ny))
                    q.append((nx, ny))

    return False


def shortest_path_length(grid):
    start = find_symbol(grid, "S")
    goal = find_symbol(grid, "G")
    if start is None or goal is None:
        return -1

    height = len(grid)
    width = len(grid[0])

    q = deque([(start[0], start[1], 0)])
    visited = {start}
    directions = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while q:
        x, y, d = q.popleft()
        if (x, y) == goal:
            return d

        for dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                if (nx, ny) not in visited and grid[ny][nx] != "#":
                    visited.add((nx, ny))
                    q.append((nx, ny, d + 1))

    return -1


def manhattan_distance(start, goal):
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])


def ensure_dirs():
    os.makedirs("maps/open", exist_ok=True)
    os.makedirs("maps/bottleneck", exist_ok=True)
    os.makedirs("maps/maze", exist_ok=True)


def clear_start_goal_cells(grid, start, goal):
    sx, sy = start
    gx, gy = goal
    grid[sy][sx] = "."
    grid[gy][gx] = "."


def generate_open_map(width=WIDTH, height=HEIGHT):
    """
    Open maps:
    - low obstacle density
    - mostly direct routes
    - a little randomness
    """
    grid = empty_grid(width, height)
    add_border_walls(grid)

    start = (1, 1)
    goal = (width - 2, height - 2)

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if (x, y) in (start, goal):
                continue
            if random.random() < 0.08:
                grid[y][x] = "#"

    # Clear a rough diagonal-ish corridor so open maps tend to remain easy
    sx, sy = start
    gx, gy = goal

    for x in range(sx, gx + 1):
        grid[sy][x] = "."

    for y in range(sy, gy + 1):
        grid[y][gx] = "."

    place_start_goal(grid, start, goal)
    return grid


def generate_bottleneck_map(width=WIDTH, height=HEIGHT):
    """
    Bottleneck maps:
    - large separating walls
    - one forced crossing point
    - often causes a real detour
    """
    grid = empty_grid(width, height)
    add_border_walls(grid)

    start = (2, 2)
    goal = (width - 3, height - 3)

    # Create a strong vertical wall barrier
    wall_x = width // 2

    for y in range(1, height - 1):
        grid[y][wall_x] = "#"

    # Put the single gap far from the straight-line route to force detour
    possible_gap_rows = list(range(height // 2 + 2, height - 2))
    gap_y = random.choice(possible_gap_rows)
    grid[gap_y][wall_x] = "."

    # Add light clutter on each side without blocking the main forced crossing
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if x == wall_x:
                continue
            if (x, y) in (start, goal):
                continue
            if abs(y - gap_y) <= 1 and abs(x - wall_x) <= 2:
                continue
            if random.random() < 0.10:
                grid[y][x] = "#"

    # Clear access around the forced gap
    for dx in [-2, -1, 0, 1, 2]:
        nx = wall_x + dx
        if 1 <= nx < width - 1:
            grid[gap_y][nx] = "."

    for dy in [-1, 0, 1]:
        ny = gap_y + dy
        if 1 <= ny < height - 1:
            grid[ny][wall_x - 1] = "."
            grid[ny][wall_x + 1] = "."

    clear_start_goal_cells(grid, start, goal)
    place_start_goal(grid, start, goal)
    return grid


def carve_corridor(grid, x1, y1, x2, y2):
    """
    Carves an L-shaped corridor between two points.
    """
    x, y = x1, y1
    grid[y][x] = "."

    if random.random() < 0.5:
        while x != x2:
            x += 1 if x2 > x else -1
            grid[y][x] = "."
        while y != y2:
            y += 1 if y2 > y else -1
            grid[y][x] = "."
    else:
        while y != y2:
            y += 1 if y2 > y else -1
            grid[y][x] = "."
        while x != x2:
            x += 1 if x2 > x else -1
            grid[y][x] = "."


def generate_maze_like_map(width=WIDTH, height=HEIGHT):
    """
    Maze-like maps:
    - start from mostly blocked interior
    - carve winding corridor segments
    - add branches / dead ends
    - produce more variable path lengths
    """
    grid = empty_grid(width, height, fill="#")
    add_border_walls(grid)

    start = (1, 1)
    goal = (width - 2, height - 2)

    # Open all interior? No. Start from blocked and carve paths.
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            grid[y][x] = "#"

    # Main winding route through intermediate waypoints
    waypoints = [
        start,
        (random.randint(2, width // 3), random.randint(height // 3, height - 4)),
        (random.randint(width // 3, 2 * width // 3), random.randint(2, height // 2)),
        (random.randint(2 * width // 3, width - 3), random.randint(height // 2, height - 3)),
        goal,
    ]

    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        carve_corridor(grid, x1, y1, x2, y2)

    # Widen some corridor cells slightly and add side branches
    open_cells = []
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if grid[y][x] == ".":
                open_cells.append((x, y))

    # Small widening to avoid a single rigid hallway everywhere
    for x, y in random.sample(open_cells, min(len(open_cells), 20)):
        for dx, dy in random.sample([(1, 0), (-1, 0), (0, 1), (0, -1)], 2):
            nx, ny = x + dx, y + dy
            if 1 <= nx < width - 1 and 1 <= ny < height - 1:
                grid[ny][nx] = "."

    # Add short dead-end branches
    branch_starts = random.sample(open_cells, min(len(open_cells), 12))
    for bx, by in branch_starts:
        dx, dy = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
        length = random.randint(2, 5)

        x, y = bx, by
        for _ in range(length):
            x += dx
            y += dy
            if not (1 <= x < width - 1 and 1 <= y < height - 1):
                break
            grid[y][x] = "."

    clear_start_goal_cells(grid, start, goal)
    place_start_goal(grid, start, goal)
    return grid


def generate_until_valid(generator, family_name, min_extra_detour=0, max_attempts=500):
    """
    Repeatedly generate until:
    - map is solvable
    - shortest path is at least Manhattan(start, goal) + min_extra_detour
    """
    for attempt in range(max_attempts):
        if attempt % 50 == 0:
            print(f"Generating {family_name}: attempt {attempt}")

        grid = generator()

        if not is_solvable(grid):
            continue

        start = find_symbol(grid, "S")
        goal = find_symbol(grid, "G")
        if start is None or goal is None:
            continue

        base = manhattan_distance(start, goal)
        sp = shortest_path_length(grid)

        if sp == -1:
            continue

        if sp >= base + min_extra_detour:
            return grid

    raise RuntimeError(f"Failed to generate a valid {family_name} map after {max_attempts} attempts.")


def main():
    random.seed(SEED)
    ensure_dirs()

    # Open: usually near-direct paths
    for i in range(OPEN_COUNT):
        grid = generate_until_valid(generate_open_map, "open", min_extra_detour=0)
        save_grid(grid, f"maps/open/open_{i+1}.txt")

    # Bottleneck: should often force at least some detour
    for i in range(BOTTLENECK_COUNT):
        grid = generate_until_valid(generate_bottleneck_map, "bottleneck", min_extra_detour=2)
        save_grid(grid, f"maps/bottleneck/bottleneck_{i+1}.txt")

    # Maze-like: should typically force larger detours
    for i in range(MAZE_COUNT):
        grid = generate_until_valid(generate_maze_like_map, "maze", min_extra_detour=4)
        save_grid(grid, f"maps/maze/maze_{i+1}.txt")

    print("Maps generated.")


if __name__ == "__main__":
    main()
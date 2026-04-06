import os
import random
from collections import deque


WIDTH = 40
HEIGHT = 40
SEED = 42

OPEN_COUNT = 30
BOTTLENECK_COUNT = 30
MAZE_COUNT = 30
DECEPTIVE_COUNT = 30

MIN_START_GOAL_MANHATTAN = 10
MAX_PAIR_SAMPLING_ATTEMPTS = 300
MAX_MAP_ATTEMPTS = 500

OPEN_DETOUR_MIN = 1.00
OPEN_DETOUR_MAX = 1.20

BOTTLENECK_DETOUR_MIN = 1.10
BOTTLENECK_DETOUR_MAX = 2.20

MAZE_DETOUR_MIN = 1.30
MAZE_DETOUR_MAX = 4.00

DECEPTIVE_DETOUR_MIN = 1.50
DECEPTIVE_DETOUR_MAX = 4.50


def empty_grid(width, height, fill="."):
    grid = []

    for _ in range(height):
        row = []
        for _ in range(width):
            row.append(fill)
        grid.append(row)

    return grid


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


def clear_start_goal(grid):
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell in ("S", "G"):
                grid[y][x] = "."


def find_symbol(grid, symbol):
    for y, row in enumerate(grid):
        for x, cell in enumerate(row):
            if cell == symbol:
                return (x, y)
    return None


def is_solvable(grid):
    """
    Runs BFS to check if grid is solvable
    """
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
    os.makedirs("maps/deceptive", exist_ok=True)


def interior_open_cells(grid):
    cells = []
    height = len(grid)
    width = len(grid[0])

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if grid[y][x] == ".":
                cells.append((x, y))

    return cells


def choose_start_goal(grid, min_manhattan=MIN_START_GOAL_MANHATTAN, attempts=MAX_PAIR_SAMPLING_ATTEMPTS):
    cells = interior_open_cells(grid)
    if len(cells) < 2:
        return None, None

    for _ in range(attempts):
        start = random.choice(cells)
        goal = random.choice(cells)

        if start == goal:
            continue

        if manhattan_distance(start, goal) < min_manhattan:
            continue

        return start, goal

    return None, None


def carve_corridor(grid, x1, y1, x2, y2):
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


def add_random_openings(grid, count):
    height = len(grid)
    width = len(grid[0])

    for _ in range(count):
        x = random.randint(1, width - 2)
        y = random.randint(1, height - 2)
        grid[y][x] = "."


def add_random_blocks(grid, density):
    height = len(grid)
    width = len(grid[0])

    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if random.random() < density:
                grid[y][x] = "#"


def generate_open_environment(width=WIDTH, height=HEIGHT):
    """
    Sparse random obstacles.
    Usually heuristic-friendly, but start/goal are sampled later.
    """
    grid = empty_grid(width, height)
    add_border_walls(grid)

    add_random_blocks(grid, density=0.10)

    # Light smoothing: clear some random cells to avoid cluttered ugliness
    add_random_openings(grid, count=(width * height) // 15)

    return grid


def generate_bottleneck_environment(width=WIDTH, height=HEIGHT):
    """
    Two broad regions separated by a strong barrier with 1-2 narrow crossings.
    Designed so some sampled pairs favor A* and some favor MM.
    """
    grid = empty_grid(width, height)
    add_border_walls(grid)

    # Place a major vertical or horizontal separator
    orientation = random.choice(["vertical", "horizontal"])

    if orientation == "vertical":
        # Putting the wall roughly near the middle gives two meaningful sides
        wall_x = random.randint(width // 3, 2 * width // 3)

        for y in range(1, height - 1):
            grid[y][wall_x] = "#"

        gap_count = random.choice([1, 2])

        if gap_count == 1:
            # Choose one row somewhere not too close to the border
            gap_positions = [random.randint(2, height - 3)]
        else:
            # Put two gaps that are far apart
            top_gap = random.randint(2, height // 3)
            bottom_gap = random.randint(2 * height // 3, height - 3)
            gap_positions = [top_gap, bottom_gap]

        for gy in gap_positions:
            # Makes a crossing point
            grid[gy][wall_x] = "."

            # Makes a 3x3 open patch centered at the gap, as long as it stays inside the interior
            for dy in [-1, 0, 1]:
                ny = gy + dy
                if 1 <= ny < height - 1:
                    for dx in [-1, 0, 1]:
                        nx = wall_x + dx
                        if 1 <= nx < width - 1:
                            grid[ny][nx] = "."

        # Light clutter away from separator
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if abs(x - wall_x) <= 1:
                    continue
                if random.random() < 0.08:
                    grid[y][x] = "#"

    else:
        wall_y = random.randint(height // 3, 2 * height // 3)

        for x in range(1, width - 1):
            grid[wall_y][x] = "#"

        gap_count = random.choice([1, 2])

        if gap_count == 1:
            gap_positions = [random.randint(2, width - 3)]
        else:
            left_gap = random.randint(2, width // 3)
            right_gap = random.randint(2 * width // 3, width - 3)
            gap_positions = [left_gap, right_gap]

        for gx in gap_positions:
            grid[wall_y][gx] = "."

            for dx in [-1, 0, 1]:
                nx = gx + dx
                if 1 <= nx < width - 1:
                    for dy in [-1, 0, 1]:
                        ny = wall_y + dy
                        if 1 <= ny < height - 1:
                            grid[ny][nx] = "."

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if abs(y - wall_y) <= 1:
                    continue
                if random.random() < 0.08:
                    grid[y][x] = "#"

    return grid


def generate_maze_environment(width=WIDTH, height=HEIGHT):
    """
    Corridor-heavy environment with branches.
    More likely to reduce Manhattan's usefulness.
    """
    grid = empty_grid(width, height, fill="#")
    add_border_walls(grid)

    # Carve a network of corridors via random waypoints
    waypoints = []
    for _ in range(7):
        x = random.randint(1, width - 2)
        y = random.randint(1, height - 2)
        waypoints.append((x, y))

    for i in range(len(waypoints) - 1):
        x1, y1 = waypoints[i]
        x2, y2 = waypoints[i + 1]
        carve_corridor(grid, x1, y1, x2, y2)

    # Add some extra branches to create little dead ends or side corridors
    open_cells = interior_open_cells(grid)
    if open_cells:
        for bx, by in random.sample(open_cells, min(len(open_cells), 20)):
            dx, dy = random.choice([(1, 0), (-1, 0), (0, 1), (0, -1)])
            length = random.randint(2, 6)

            x, y = bx, by
            for _ in range(length):
                x += dx
                y += dy
                if not (1 <= x < width - 1 and 1 <= y < height - 1):
                    break
                grid[y][x] = "."

    # Slight widening here and there
    open_cells = interior_open_cells(grid)
    if open_cells:
        for x, y in random.sample(open_cells, min(len(open_cells), 25)):
            for dx, dy in random.sample([(1, 0), (-1, 0), (0, 1), (0, -1)], 2):
                nx, ny = x + dx, y + dy
                if 1 <= nx < width - 1 and 1 <= ny < height - 1:
                    grid[ny][nx] = "."

    return grid


def generate_deceptive_environment(width=WIDTH, height=HEIGHT):
    """
    Environments meant to make Manhattan misleading:
    near-looking targets blocked by walls, U-shapes, and offset openings.
    """
    grid = empty_grid(width, height)
    add_border_walls(grid)

    # Each map gets between 10 and 15 obstacle patterns
    pattern_count = random.randint(10, 15)

    for _ in range(pattern_count):
        pattern = random.choice(["long_wall", "u_shape", "box_with_gap"])

        if pattern == "long_wall":
            orientation = random.choice(["horizontal", "vertical"])

            if orientation == "horizontal":
                y = random.randint(3, height - 4)
                x1 = random.randint(2, width // 3)
                x2 = random.randint(2 * width // 3, width - 3)

                for x in range(x1, x2 + 1):
                    grid[y][x] = "#"

                # gap near one end, not near center
                gap_x = random.choice([x1, x1 + 1, x2 - 1, x2])
                grid[y][gap_x] = "."

            else:
                x = random.randint(3, width - 4)
                y1 = random.randint(2, height // 3)
                y2 = random.randint(2 * height // 3, height - 3)

                for y in range(y1, y2 + 1):
                    grid[y][x] = "#"

                gap_y = random.choice([y1, y1 + 1, y2 - 1, y2])
                grid[gap_y][x] = "."

        elif pattern == "u_shape":
            left = random.randint(2, width - 8)
            top = random.randint(2, height - 8)
            w = random.randint(4, 6)
            h = random.randint(4, 6)
            open_side = random.choice(["top", "bottom", "left", "right"])

            if open_side != "top":
                for x in range(left, left + w):
                    grid[top][x] = "#"
            if open_side != "bottom":
                for x in range(left, left + w):
                    grid[top + h - 1][x] = "#"
            if open_side != "left":
                for y in range(top, top + h):
                    grid[y][left] = "#"
            if open_side != "right":
                for y in range(top, top + h):
                    grid[y][left + w - 1] = "#"

        else:  # box_with_gap
            left = random.randint(2, width - 8)
            top = random.randint(2, height - 8)
            w = random.randint(4, 6)
            h = random.randint(4, 6)

            for x in range(left, left + w):
                grid[top][x] = "#"
                grid[top + h - 1][x] = "#"
            for y in range(top, top + h):
                grid[y][left] = "#"
                grid[y][left + w - 1] = "#"

            side = random.choice(["top", "bottom", "left", "right"])
            if side == "top":
                gx = random.choice([left + 1, left + w - 2])
                grid[top][gx] = "."
            elif side == "bottom":
                gx = random.choice([left + 1, left + w - 2])
                grid[top + h - 1][gx] = "."
            elif side == "left":
                gy = random.choice([top + 1, top + h - 2])
                grid[gy][left] = "."
            else:
                gy = random.choice([top + 1, top + h - 2])
                grid[gy][left + w - 1] = "."

    # Light background clutter
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if grid[y][x] == "." and random.random() < 0.04:
                grid[y][x] = "#"

    # Reopen a few random cells so clutter does not overblock
    add_random_openings(grid, count=(width * height) // 22)

    return grid


def classify_pair(grid, start, goal):
    clear_start_goal(grid)
    place_start_goal(grid, start, goal)

    if not is_solvable(grid):
        clear_start_goal(grid)
        return None

    sp = shortest_path_length(grid)
    if sp == -1:
        clear_start_goal(grid)
        return None

    md = manhattan_distance(start, goal)
    if md == 0:
        clear_start_goal(grid)
        return None

    ratio = sp / md
    clear_start_goal(grid)

    return {
        "shortest_path": sp,
        "manhattan": md,
        "detour_ratio": ratio,
    }


def generate_until_valid(
    environment_generator,
    family_name,
    detour_min,
    detour_max,
    max_attempts=MAX_MAP_ATTEMPTS,
):
    """
    Generate an environment, then sample a start-goal pair whose
    detour ratio falls in the family's target range.
    """
    for attempt in range(max_attempts):

        grid = environment_generator()

        start, goal = choose_start_goal(grid)
        if start is None or goal is None:
            continue

        info = classify_pair(grid, start, goal)
        if info is None:
            continue

        ratio = info["detour_ratio"]

        if detour_min <= ratio <= detour_max:
            place_start_goal(grid, start, goal)
            return grid

    raise RuntimeError(
        f"Failed to generate a valid {family_name} map after {max_attempts} attempts."
    )


def main():
    random.seed(SEED)
    ensure_dirs()

    for i in range(OPEN_COUNT):
        grid = generate_until_valid(
            generate_open_environment,
            "open",
            detour_min=OPEN_DETOUR_MIN,
            detour_max=OPEN_DETOUR_MAX,
        )
        save_grid(grid, f"maps/open/open_{i+1}.txt")

    for i in range(BOTTLENECK_COUNT):
        grid = generate_until_valid(
            generate_bottleneck_environment,
            "bottleneck",
            detour_min=BOTTLENECK_DETOUR_MIN,
            detour_max=BOTTLENECK_DETOUR_MAX,
        )
        save_grid(grid, f"maps/bottleneck/bottleneck_{i+1}.txt")

    for i in range(MAZE_COUNT):
        grid = generate_until_valid(
            generate_maze_environment,
            "maze",
            detour_min=MAZE_DETOUR_MIN,
            detour_max=MAZE_DETOUR_MAX,
        )
        save_grid(grid, f"maps/maze/maze_{i+1}.txt")

    for i in range(DECEPTIVE_COUNT):
        grid = generate_until_valid(
            generate_deceptive_environment,
            "deceptive",
            detour_min=DECEPTIVE_DETOUR_MIN,
            detour_max=DECEPTIVE_DETOUR_MAX,
        )
        save_grid(grid, f"maps/deceptive/deceptive_{i+1}.txt")

    print("Maps generated.")


if __name__ == "__main__":
    main()
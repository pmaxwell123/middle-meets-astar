from algorithms import State


class GridMap:
    """
    Simple grid map for single-agent search experiments.

    Symbols:
        S = start
        G = goal
        . = free cell
        # = blocked cell
    """

    def __init__(self, file_name):
        self.file_name = file_name
        self.grid = []
        self.start = None
        self.goal = None

        self._read_grid()

        self.height = len(self.grid)
        self.width = len(self.grid[0])

        State.map_width = self.width
        State.map_height = self.height

    def _read_grid(self):
        """
        Reads a simple grid file into memory.
        """
        with open(self.file_name, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f if line.strip()]

        if not lines:
            raise ValueError("Grid file is empty.")

        row_length = len(lines[0])
        for line in lines:
            if len(line) != row_length:
                raise ValueError("All rows in the grid must have the same length.")

        valid_symbols = {"S", "G", ".", "#"}

        for y, line in enumerate(lines):
            row = []
            for x, ch in enumerate(line):
                if ch not in valid_symbols:
                    raise ValueError(f"Invalid grid symbol: {ch}")

                if ch == "S":
                    if self.start is not None:
                        raise ValueError("Grid cannot contain more than one start 'S'.")
                    self.start = State(x, y)

                if ch == "G":
                    if self.goal is not None:
                        raise ValueError("Grid cannot contain more than one goal 'G'.")
                    self.goal = State(x, y)

                row.append(ch)
            self.grid.append(row)

        if self.start is None:
            raise ValueError("Grid must contain a start 'S'.")
        if self.goal is None:
            raise ValueError("Grid must contain a goal 'G'.")

    def is_valid_pair(self, x, y):
        """
        Returns True if (x, y) is inside the grid and not blocked.
        """
        if x < 0 or y < 0:
            return False
        if x >= self.width or y >= self.height:
            return False
        if self.grid[y][x] == "#":
            return False
        return True

    def successors(self, state):
        """
        Returns 4-direction neighbors of the given state.
        Each move has cost 1.
        """
        children = []

        directions = [
            (0, -1),   # up
            (0, 1),    # down
            (-1, 0),   # left
            (1, 0),    # right
        ]

        for dx, dy in directions:
            new_x = state.get_x() + dx
            new_y = state.get_y() + dy

            if self.is_valid_pair(new_x, new_y):
                s = State(new_x, new_y)
                s.set_g(state.get_g() + 1)
                children.append(s)

        return children

    def get_start(self):
        return self.start

    def get_goal(self):
        return self.goal

    def print_map(self):
        for row in self.grid:
            print("".join(row))
import heapq


class State:
    """
    Represents a state in a grid as (x, y).

    Static variables map_width and map_height are used to compute a unique hash
    value for each state.
    """
    map_width = 0
    map_height = 0

    def __init__(self, x, y):
        self._x = x
        self._y = y
        self._g = 0
        self._cost = 0
        self._parent = None

    def __repr__(self):
        return f"[{self._x}, {self._y}]"

    def __lt__(self, other):
        return self._cost < other._cost

    def __eq__(self, other):
        return self._x == other._x and self._y == other._y

    def state_hash(self):
        return self._y * State.map_width + self._x

    def get_x(self):
        return self._x

    def get_y(self):
        return self._y

    def get_g(self):
        return self._g

    def set_g(self, g):
        self._g = g

    def get_cost(self):
        return self._cost

    def set_cost(self, cost):
        self._cost = cost

    def get_parent(self):
        return self._parent

    def set_parent(self, parent):
        self._parent = parent


def reconstruct_path(goal):
    path = []
    node = goal

    while node is not None:
        path.append(node)
        node = node.get_parent()

    path.reverse()
    return path


def manhattan(state, goal):
    return abs(state.get_x() - goal.get_x()) + abs(state.get_y() - goal.get_y())


def astar(grid_map, start, goal):
    heap_open = []
    best_g = {}

    start.set_g(0)
    start.set_parent(None)
    start.set_cost(manhattan(start, goal))

    heapq.heappush(heap_open, start)
    best_g[start.state_hash()] = 0

    expansions = 0
    goal_hash = goal.state_hash()

    while heap_open:
        current = heapq.heappop(heap_open)
        current_hash = current.state_hash()

        if current.get_g() > best_g.get(current_hash, float("inf")):
            continue

        expansions += 1

        if current_hash == goal_hash:
            path = reconstruct_path(current)
            return path, current.get_g(), expansions

        for neighbor in grid_map.successors(current):
            neighbor_hash = neighbor.state_hash()
            new_g = neighbor.get_g()

            if new_g < best_g.get(neighbor_hash, float("inf")):
                best_g[neighbor_hash] = new_g
                neighbor.set_parent(current)
                neighbor.set_cost(new_g + manhattan(neighbor, goal))
                heapq.heappush(heap_open, neighbor)

    return None, -1, expansions


def _make_state_from_hash(state_hash):
    x = state_hash % State.map_width
    y = state_hash // State.map_width
    return State(x, y)


def _reconstruct_mm_path(meeting_hash, parent_f, parent_b):
    """
    Reconstruct full path start -> ... -> meeting -> ... -> goal
    using forward parents and backward parents.
    """
    # Forward half: meeting back to start
    forward_hashes = []
    h = meeting_hash
    while h is not None:
        forward_hashes.append(h)
        h = parent_f.get(h)
    forward_hashes.reverse()

    # Backward half: from meeting to goal
    backward_hashes = []
    h = parent_b.get(meeting_hash)
    while h is not None:
        backward_hashes.append(h)
        h = parent_b.get(h)

    full_hashes = forward_hashes + backward_hashes
    return [_make_state_from_hash(h) for h in full_hashes]


def mm(grid_map, start, goal, epsilon=1):
    """
    Meet-in-the-Middle bidirectional heuristic search, closer to the lecture pseudocode.

    Uses:
        p(n) = max(f(n), 2g(n))

    Returns:
        path: list of State objects from start to goal
        cost: optimal path cost
        expansions: total node expansions across both directions
    """
    start_hash = start.state_hash()
    goal_hash = goal.state_hash()

    if start_hash == goal_hash:
        return [State(start.get_x(), start.get_y())], 0, 0

    # OPEN heaps store tuples: (p, f, g, state_hash)
    open_f = []
    open_b = []

    # Best known g-values in each direction
    g_f = {start_hash: 0}
    g_b = {goal_hash: 0}

    # Parent pointers
    parent_f = {start_hash: None}
    parent_b = {goal_hash: None}

    # CLOSED dictionaries: store expanded states and their g-values
    # This makes "if n in CLOSED then use CLOSED[n].g" match the pseudocode more closely.
    closed_f = {}
    closed_b = {}

    # Initial forward state
    f_start = manhattan(start, goal)
    p_start = max(f_start, 0)
    heapq.heappush(open_f, (p_start, f_start, 0, start_hash))

    # Initial backward state
    f_goal = manhattan(goal, start)
    p_goal = max(f_goal, 0)
    heapq.heappush(open_b, (p_goal, f_goal, 0, goal_hash))

    U = float("inf")
    meeting_hash = None
    expansions = 0

    def peek_valid(heap, g_dir):
        """
        Return the best valid (p, f, g, hash) from a heap without removing it.
        Discards stale entries.
        """
        while heap:
            p, f, g, hsh = heap[0]
            if g == g_dir.get(hsh, float("inf")):
                return p, f, g, hsh
            heapq.heappop(heap)
        return float("inf"), float("inf"), float("inf"), None

    while open_f and open_b:
        p_min_f, f_min_f, g_min_f, _ = peek_valid(open_f, g_f)
        p_min_b, f_min_b, g_min_b, _ = peek_valid(open_b, g_b)

        lower_bound = max(
            g_min_f + g_min_b + epsilon,
            f_min_f,
            f_min_b,
            min(p_min_f, p_min_b),
        )

        # Stopping condition from the lecture pseudocode/slides. :contentReference[oaicite:1]{index=1}
        if U <= lower_bound:
            if meeting_hash is None:
                return None, -1, expansions
            path = _reconstruct_mm_path(meeting_hash, parent_f, parent_b)
            return path, U, expansions

        expand_forward = p_min_f <= p_min_b

        if expand_forward:
            p_cur, f_cur, g_cur, cur_hash = heapq.heappop(open_f)

            # stale entry
            if g_cur != g_f.get(cur_hash, float("inf")):
                continue

            current = _make_state_from_hash(cur_hash)
            current.set_g(g_cur)

            # Add current to CLOSED_f
            closed_f[cur_hash] = g_cur
            expansions += 1

            for neighbor in grid_map.successors(current):
                n_hash = neighbor.state_hash()
                new_g = neighbor.get_g()

                # If neighbor is in opposite CLOSED, update upper bound U
                if n_hash in closed_b:
                    candidate = new_g + closed_b[n_hash]
                    if candidate < U:
                        U = candidate
                        meeting_hash = n_hash

                # If better path found, update/reopen
                if new_g < g_f.get(n_hash, float("inf")):
                    g_f[n_hash] = new_g
                    parent_f[n_hash] = cur_hash

                    # Reopen if it had previously been closed
                    if n_hash in closed_f:
                        del closed_f[n_hash]

                    n_state = _make_state_from_hash(n_hash)
                    f_val = new_g + manhattan(n_state, goal)
                    p_val = max(f_val, 2 * new_g)
                    heapq.heappush(open_f, (p_val, f_val, new_g, n_hash))

        else:
            p_cur, f_cur, g_cur, cur_hash = heapq.heappop(open_b)

            # stale entry
            if g_cur != g_b.get(cur_hash, float("inf")):
                continue

            current = _make_state_from_hash(cur_hash)
            current.set_g(g_cur)

            # Add current to CLOSED_b
            closed_b[cur_hash] = g_cur
            expansions += 1

            for neighbor in grid_map.successors(current):
                n_hash = neighbor.state_hash()
                new_g = neighbor.get_g()

                # If neighbor is in opposite CLOSED, update upper bound U
                if n_hash in closed_f:
                    candidate = new_g + closed_f[n_hash]
                    if candidate < U:
                        U = candidate
                        meeting_hash = n_hash

                # If better path found, update/reopen
                if new_g < g_b.get(n_hash, float("inf")):
                    g_b[n_hash] = new_g
                    parent_b[n_hash] = cur_hash

                    # Reopen if it had previously been closed
                    if n_hash in closed_b:
                        del closed_b[n_hash]

                    n_state = _make_state_from_hash(n_hash)
                    f_val = new_g + manhattan(n_state, start)
                    p_val = max(f_val, 2 * new_g)
                    heapq.heappush(open_b, (p_val, f_val, new_g, n_hash))

    if meeting_hash is not None:
        path = _reconstruct_mm_path(meeting_hash, parent_f, parent_b)
        return path, U, expansions

    return None, -1, expansions
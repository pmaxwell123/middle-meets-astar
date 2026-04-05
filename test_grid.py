from grid_map import GridMap
from algorithms import astar, mm

m = GridMap("maps/maze/maze_2.txt")
start = m.get_start()
goal = m.get_goal()

path_a, cost_a, exp_a = astar(m, start, goal)
print("A*")
print("Path:", path_a)
print("Cost:", cost_a)
print("Expansions:", exp_a)

# Re-read map so start/goal are fresh objects
m2 = GridMap("maps/maze/maze_2.txt")
start2 = m2.get_start()
goal2 = m2.get_goal()

path_m, cost_m, exp_m = mm(m2, start2, goal2, epsilon=1)
print("\nMM")
print("Path:", path_m)
print("Cost:", cost_m)
print("Expansions:", exp_m)
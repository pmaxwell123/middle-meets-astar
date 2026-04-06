# middle-meets-astar

Exploration of A*, Meet-in-the-Middle (MM), and simulated dovetailing through grid-based pathfinding experiments.

This project studies how different search strategies behave across families of single-agent grid maps, with a focus on **node expansions** as the main performance metric. It compares:

- **A\***  
- **Meet-in-the-Middle (MM)**  
- **Simulated dovetailing** between A* and MM under multiple budget allocation ratios  

The project is motivated by the question:

> In single-agent grid search across open, bottleneck, maze-like, and deceptive environments, how do search strategy (MM, A*, or simulated dovetailing) and dovetailing budget allocation ratio affect the number of node expansions?

---

## Overview

The repository contains code to:

- load grid maps from experiment folders
- run **A\*** and **MM** on each map
- simulate **dovetailing** post hoc using completed standalone runs
- save experiment results to `results.csv`
- print aggregate summaries
- generate visualizations comparing search behavior across map families

The current experiment setup uses map families such as:

- `open`
- `bottleneck`
- `maze`
- `deceptive` (included in the runner, optional in analysis depending on thesis scope)

`main.py` iterates through map files, runs A* and MM, computes simulated dovetailing results for several ratios, and writes everything to CSV.

### Instructions
1. Install the latest version of matplotlib
2. Run generate_maps.py
3. Run main.py
4. Run analyze_results.py
---

## Search Strategies

### A*
A classic informed search algorithm that expands nodes according to:
- path cost so far
- heuristic estimate to the goal

### Meet-in-the-Middle (MM)
A bidirectional search approach that searches from both the start and the goal, attempting to reduce work by meeting in the middle.

### Simulated dovetailing
This project also evaluates a **post hoc simulated dovetailing** scheme. Rather than truly interleaving the searches during execution, it takes the completed expansion counts of standalone A* and MM runs and estimates which search would finish first under round-based resource allocation.

Current allocation ratios tested are:

- `1:1`
- `2:1`
- `1:2`
- `3:1`
- `1:3`

These are interpreted as **A*:MM expansion budgets per round**.

---

## Repository Structure

```text
.
├── main.py
├── analyze_results.py
├── maps/
│   ├── open/
│   ├── bottleneck/
│   ├── maze/
│   └── deceptive/
├── grid_map.py
├── algorithms.py
├── test_grid.py
└── results.csv
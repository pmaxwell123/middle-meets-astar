import csv
import os
import math

from grid_map import GridMap
from algorithms import astar, mm


MAP_ROOT = "maps"
OUTPUT_CSV = "results.csv"
FAMILIES = ["open", "bottleneck", "maze", "deceptive"]

DOVETAIL_RATIOS = [
    (1, 1),
    (2, 1),
    (1, 2),
    (3, 1),
    (1, 3),
]


def simulate_dovetail_from_finished_runs(astar_expansions, mm_expansions, astar_budget, mm_budget):
    """
    Post hoc simulated dovetailing.

    We do not truly interleave the searches. Instead, we use each algorithm's
    standalone expansion count and ask which one would finish first if each
    round allocated:
        astar_budget expansions to A*
        mm_budget expansions to MM
    """
    if astar_expansions == -1 and mm_expansions == -1:
        return {
            "winner": "n/a",
            "rounds": -1,
            "total_expansions": -1,
        }

    astar_rounds = math.inf if astar_expansions == -1 else math.ceil(astar_expansions / astar_budget)
    mm_rounds = math.inf if mm_expansions == -1 else math.ceil(mm_expansions / mm_budget)

    if astar_rounds < mm_rounds:
        winner = "A*"
        rounds = astar_rounds
    elif mm_rounds < astar_rounds:
        winner = "MM"
        rounds = mm_rounds
    else:
        winner = "tie"
        rounds = astar_rounds

    total_expansions = rounds * (astar_budget + mm_budget)

    return {
        "winner": winner,
        "rounds": rounds,
        "total_expansions": total_expansions,
    }

def iter_map_files(root=MAP_ROOT):
    """
    Yields (family, filename, full_path) for all .txt map files under:
        maps/open
        maps/bottleneck
        maps/maze
        maps/deceptive
    """
    for family in FAMILIES:
        family_dir = os.path.join(root, family)
        if not os.path.isdir(family_dir):
            continue

        for filename in sorted(os.listdir(family_dir)):
            if filename.endswith(".txt"):
                yield family, filename, os.path.join(family_dir, filename)


def run_single_map(map_path):
    """
    Runs A* and MM on a single map and returns a result dict.
    """
    # Fresh map/state objects for A*
    map_a = GridMap(map_path)
    start_a = map_a.get_start()
    goal_a = map_a.get_goal()

    path_a, cost_a, expansions_a = astar(map_a, start_a, goal_a)

    # Fresh map/state objects for MM
    map_m = GridMap(map_path)
    start_m = map_m.get_start()
    goal_m = map_m.get_goal()

    path_m, cost_m, expansions_m = mm(map_m, start_m, goal_m, epsilon=1)

    dovetail_results = {}

    if path_a is not None or path_m is not None:
        for astar_budget, mm_budget in DOVETAIL_RATIOS:
            sim = simulate_dovetail_from_finished_runs(
                expansions_a,
                expansions_m,
                astar_budget,
                mm_budget,
            )

            ratio_name = f"dovetail_{astar_budget}_{mm_budget}"
            dovetail_results[f"{ratio_name}_winner"] = sim["winner"]
            dovetail_results[f"{ratio_name}_rounds"] = sim["rounds"]
            dovetail_results[f"{ratio_name}_total_expansions"] = sim["total_expansions"]
    else:
        for astar_budget, mm_budget in DOVETAIL_RATIOS:
            ratio_name = f"dovetail_{astar_budget}_{mm_budget}"
            dovetail_results[f"{ratio_name}_winner"] = "n/a"
            dovetail_results[f"{ratio_name}_rounds"] = -1
            dovetail_results[f"{ratio_name}_total_expansions"] = -1

    cost_match = cost_a == cost_m
    solvable_match = (
        (path_a is None and path_m is None)
        or (path_a is not None and path_m is not None)
    )

    if path_a is not None and path_m is not None:
        expansion_diff = expansions_m - expansions_a

        if expansions_a > 0:
            expansion_ratio = expansions_m / expansions_a
        else:
            expansion_ratio = None

        if expansions_a < expansions_m:
            winner = "A*"
        elif expansions_m < expansions_a:
            winner = "MM"
        else:
            winner = "tie"
    else:
        expansion_diff = None
        expansion_ratio = None
        winner = "n/a"

    result = {
        "map_path": map_path,
        "astar_path_length": len(path_a) if path_a is not None else 0,
        "astar_cost": cost_a,
        "astar_expansions": expansions_a,
        "mm_path_length": len(path_m) if path_m is not None else 0,
        "mm_cost": cost_m,
        "mm_expansions": expansions_m,
        "cost_match": cost_match,
        "solvable_match": solvable_match,
        "expansion_diff": expansion_diff,
        "expansion_ratio": expansion_ratio,
        "winner": winner,
        **dovetail_results,
    }

    return result


def print_result_row(family, filename, result):
    """
    Prints a short readable summary for one map.
    """
    print(f"\n[{family}] {filename}")
    print(
        f"A*: cost={result['astar_cost']}, "
        f"expansions={result['astar_expansions']}, "
        f"path_len={result['astar_path_length']}"
    )
    print(
        f"MM: cost={result['mm_cost']}, "
        f"expansions={result['mm_expansions']}, "
        f"path_len={result['mm_path_length']}"
    )
    print(
        f"Cost match: {result['cost_match']} | "
        f"Solvable match: {result['solvable_match']}"
    )

    if result["winner"] != "n/a":
        ratio_text = (
            f"{result['expansion_ratio']:.3f}"
            if result["expansion_ratio"] is not None
            else "n/a"
        )
        print(
            f"Winner: {result['winner']} | "
            f"MM - A* expansion diff: {result['expansion_diff']} | "
            f"MM/A* ratio: {ratio_text}"
        )
    
    print("Simulated dovetailing:")
    for astar_budget, mm_budget in DOVETAIL_RATIOS:
        ratio_name = f"dovetail_{astar_budget}_{mm_budget}"
        winner = result[f"{ratio_name}_winner"]
        rounds = result[f"{ratio_name}_rounds"]
        total_expansions = result[f"{ratio_name}_total_expansions"]

        print(
            f"  A*:MM = {astar_budget}:{mm_budget} | "
            f"winner={winner}, rounds={rounds}, total_expansions={total_expansions}"
        )


def write_results_csv(rows, output_csv=OUTPUT_CSV):
    """
    Writes experiment results to CSV.
    """
    fieldnames = [
        "family",
        "filename",
        "map_path",
        "astar_path_length",
        "astar_cost",
        "astar_expansions",
        "mm_path_length",
        "mm_cost",
        "mm_expansions",
        "cost_match",
        "solvable_match",
        "expansion_diff",
        "expansion_ratio",
        "winner",
    ]

    for astar_budget, mm_budget in DOVETAIL_RATIOS:
        ratio_name = f"dovetail_{astar_budget}_{mm_budget}"
        fieldnames.append(f"{ratio_name}_winner")
        fieldnames.append(f"{ratio_name}_rounds")
        fieldnames.append(f"{ratio_name}_total_expansions")

    with open(output_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def print_summary(rows):
    """
    Prints an aggregate summary by map family.
    """
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)

    for family in FAMILIES:
        family_rows = [row for row in rows if row["family"] == family]
        if not family_rows:
            continue

        cost_matches = sum(1 for row in family_rows if row["cost_match"])
        solvable_matches = sum(1 for row in family_rows if row["solvable_match"])

        solvable_rows = [
            row for row in family_rows
            if row["astar_cost"] != -1 and row["mm_cost"] != -1
        ]

        if solvable_rows:
            avg_astar_exp = sum(row["astar_expansions"] for row in solvable_rows) / len(solvable_rows)
            avg_mm_exp = sum(row["mm_expansions"] for row in solvable_rows) / len(solvable_rows)

            comparable_rows = [row for row in solvable_rows if row["winner"] != "n/a"]
            a_wins = sum(1 for row in comparable_rows if row["winner"] == "A*")
            mm_wins = sum(1 for row in comparable_rows if row["winner"] == "MM")
            ties = sum(1 for row in comparable_rows if row["winner"] == "tie")

            valid_ratio_rows = [
                row for row in comparable_rows
                if row["expansion_ratio"] is not None
            ]
            avg_ratio = (
                sum(row["expansion_ratio"] for row in valid_ratio_rows) / len(valid_ratio_rows)
                if valid_ratio_rows else 0
            )
        else:
            avg_astar_exp = 0
            avg_mm_exp = 0
            a_wins = 0
            mm_wins = 0
            ties = 0
            avg_ratio = 0

        print(f"\nFamily: {family}")
        print(f"Maps tested: {len(family_rows)}")
        print(f"Cost matches: {cost_matches}/{len(family_rows)}")
        print(f"Solvable matches: {solvable_matches}/{len(family_rows)}")
        print(f"Average A* expansions (solvable only): {avg_astar_exp:.2f}")
        print(f"Average MM expansions (solvable only): {avg_mm_exp:.2f}")
        print(f"A* fewer expansions: {a_wins}")
        print(f"MM fewer expansions: {mm_wins}")
        print(f"Ties: {ties}")
        print(f"Average MM/A* expansion ratio: {avg_ratio:.3f}")


def main():
    rows = []

    for family, filename, full_path in iter_map_files():
        result = run_single_map(full_path)

        row = {
            "family": family,
            "filename": filename,
            **result,
        }
        rows.append(row)

        print_result_row(family, filename, result)

    if not rows:
        print("No map files found.")
        return

    write_results_csv(rows, OUTPUT_CSV)
    print_summary(rows)

    print(f"\nResults written to {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
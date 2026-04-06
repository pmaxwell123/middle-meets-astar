import csv
from collections import defaultdict


INPUT_CSV = "results.csv"


def safe_float(numerator, denominator):
    if denominator == 0:
        return 0.0
    return numerator / denominator


def load_results(csv_path=INPUT_CSV):
    rows = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            row["astar_path_length"] = int(row["astar_path_length"])
            row["astar_cost"] = int(row["astar_cost"])
            row["astar_expansions"] = int(row["astar_expansions"])
            row["mm_path_length"] = int(row["mm_path_length"])
            row["mm_cost"] = int(row["mm_cost"])
            row["mm_expansions"] = int(row["mm_expansions"])
            row["cost_match"] = row["cost_match"] == "True"
            row["solvable_match"] = row["solvable_match"] == "True"
            rows.append(row)
    return rows


def summarize_family(rows, family_name):
    family_rows = [row for row in rows if row["family"] == family_name]

    total_maps = len(family_rows)
    cost_matches = sum(1 for row in family_rows if row["cost_match"])
    solvable_matches = sum(1 for row in family_rows if row["solvable_match"])

    solvable_rows = [
        row for row in family_rows
        if row["astar_cost"] != -1 and row["mm_cost"] != -1
    ]

    astar_wins = 0
    mm_wins = 0
    ties = 0

    total_astar_exp = 0
    total_mm_exp = 0
    total_diff = 0
    total_percent_gap = 0.0

    for row in solvable_rows:
        a = row["astar_expansions"]
        m = row["mm_expansions"]

        total_astar_exp += a
        total_mm_exp += m
        total_diff += (m - a)

        if a < m:
            astar_wins += 1
        elif m < a:
            mm_wins += 1
        else:
            ties += 1

        # positive means MM used more expansions than A*
        total_percent_gap += safe_float((m - a), a) * 100

    avg_astar_exp = safe_float(total_astar_exp, len(solvable_rows))
    avg_mm_exp = safe_float(total_mm_exp, len(solvable_rows))
    avg_diff = safe_float(total_diff, len(solvable_rows))
    avg_percent_gap = safe_float(total_percent_gap, len(solvable_rows))

    return {
        "family": family_name,
        "maps_tested": total_maps,
        "cost_matches": cost_matches,
        "solvable_matches": solvable_matches,
        "solvable_maps": len(solvable_rows),
        "avg_astar_exp": avg_astar_exp,
        "avg_mm_exp": avg_mm_exp,
        "avg_diff_mm_minus_astar": avg_diff,
        "avg_percent_gap_mm_vs_astar": avg_percent_gap,
        "astar_wins": astar_wins,
        "mm_wins": mm_wins,
        "ties": ties,
    }


def print_family_summary(summary):
    print("=" * 60)
    print(f"Family: {summary['family']}")
    print("=" * 60)
    print(f"Maps tested: {summary['maps_tested']}")
    print(f"Cost matches: {summary['cost_matches']}/{summary['maps_tested']}")
    print(f"Solvable matches: {summary['solvable_matches']}/{summary['maps_tested']}")
    print(f"Solvable maps used for expansion comparison: {summary['solvable_maps']}")
    print(f"Average A* expansions: {summary['avg_astar_exp']:.2f}")
    print(f"Average MM expansions: {summary['avg_mm_exp']:.2f}")
    print(f"Average (MM - A*) expansions: {summary['avg_diff_mm_minus_astar']:.2f}")
    print(f"Average percent gap (MM vs A*): {summary['avg_percent_gap_mm_vs_astar']:.2f}%")
    print(f"A* wins: {summary['astar_wins']}")
    print(f"MM wins: {summary['mm_wins']}")
    print(f"Ties: {summary['ties']}")
    print()


def print_overall_summary(rows):
    solvable_rows = [
        row for row in rows
        if row["astar_cost"] != -1 and row["mm_cost"] != -1
    ]

    total_maps = len(rows)
    cost_matches = sum(1 for row in rows if row["cost_match"])
    solvable_matches = sum(1 for row in rows if row["solvable_match"])

    astar_wins = 0
    mm_wins = 0
    ties = 0

    total_astar_exp = 0
    total_mm_exp = 0
    total_diff = 0
    total_percent_gap = 0.0

    for row in solvable_rows:
        a = row["astar_expansions"]
        m = row["mm_expansions"]

        total_astar_exp += a
        total_mm_exp += m
        total_diff += (m - a)

        if a < m:
            astar_wins += 1
        elif m < a:
            mm_wins += 1
        else:
            ties += 1

        total_percent_gap += safe_float((m - a), a) * 100

    print("=" * 60)
    print("OVERALL SUMMARY")
    print("=" * 60)
    print(f"Maps tested: {total_maps}")
    print(f"Cost matches: {cost_matches}/{total_maps}")
    print(f"Solvable matches: {solvable_matches}/{total_maps}")
    print(f"Average A* expansions: {safe_float(total_astar_exp, len(solvable_rows)):.2f}")
    print(f"Average MM expansions: {safe_float(total_mm_exp, len(solvable_rows)):.2f}")
    print(f"Average (MM - A*) expansions: {safe_float(total_diff, len(solvable_rows)):.2f}")
    print(f"Average percent gap (MM vs A*): {safe_float(total_percent_gap, len(solvable_rows)):.2f}%")
    print(f"A* wins: {astar_wins}")
    print(f"MM wins: {mm_wins}")
    print(f"Ties: {ties}")
    print()


def print_per_map_details(rows):
    print("=" * 60)
    print("PER-MAP DETAILS")
    print("=" * 60)

    grouped = defaultdict(list)
    for row in rows:
        grouped[row["family"]].append(row)

    for family in ["open", "bottleneck", "maze"]:
        if family not in grouped:
            continue

        print(f"\n[{family}]")
        for row in sorted(grouped[family], key=lambda r: r["filename"]):
            a = row["astar_expansions"]
            m = row["mm_expansions"]

            if a < m:
                winner = "A*"
            elif m < a:
                winner = "MM"
            else:
                winner = "Tie"

            print(
                f"{row['filename']}: "
                f"A*={a}, MM={m}, "
                f"diff(MM-A*)={m-a}, winner={winner}, "
                f"cost_match={row['cost_match']}"
            )


def main():
    rows = load_results(INPUT_CSV)

    if not rows:
        print("No results found.")
        return

    print_overall_summary(rows)

    for family in ["open", "bottleneck", "maze"]:
        summary = summarize_family(rows, family)
        if summary["maps_tested"] > 0:
            print_family_summary(summary)

    print_per_map_details(rows)


if __name__ == "__main__":
    main()
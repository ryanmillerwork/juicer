#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import json
from typing import Dict, List, Tuple

import matplotlib.pyplot as plt

CSV_DIR = Path("/mnt/analysis/data/juicer_calib")
FIG_DIR = Path("/mnt/analysis/figures/juicer_calib")


def load_csv_rows(path: Path) -> List[Dict[str, str]]:
    """Return all rows from the CSV as a list of dicts."""
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV file is missing a header row.")
        return list(reader)


def extract_series(rows: List[Dict[str, str]]) -> Tuple[List[float], List[float], List[float], List[float]]:
    """Return series for reward_mls, reward_number, and juice_level A4/A5."""
    rewards: List[float] = []
    reward_numbers: List[float] = []
    juice_levels_a4: List[float] = []
    juice_levels_a5: List[float] = []

    for row in rows:
        try:
            reward = float(row["reward_mls"])
            reward_number = float(row["reward_number"])
        except (KeyError, ValueError):
            # Skip rows that are missing or have invalid numeric data.
            continue

        try:
            raw_level = row["juice_level"]
        except KeyError:
            continue

        parsed_levels = parse_juice_levels(raw_level)
        if parsed_levels is None:
            continue
        level_a4, level_a5 = parsed_levels

        rewards.append(reward)
        reward_numbers.append(reward_number)
        juice_levels_a4.append(level_a4)
        juice_levels_a5.append(level_a5)

    if not rewards:
        raise ValueError("No valid rows found to plot.")

    return rewards, reward_numbers, juice_levels_a4, juice_levels_a5


def parse_juice_levels(value: str) -> Tuple[float, float] | None:
    """Parse the juice_level column and return the A4 and A5 readings."""
    if value is None:
        return None

    value = value.strip()
    if not value:
        return None

    try:
        parsed = json.loads(value)
    except json.JSONDecodeError:
        try:
            float_val = float(value)
        except ValueError:
            return None
        return float_val, float_val

    if isinstance(parsed, (list, tuple)):
        if len(parsed) >= 3:
            try:
                return float(parsed[1]), float(parsed[2])
            except (TypeError, ValueError):
                return None
        if len(parsed) == 1:
            try:
                float_val = float(parsed[0])
                return float_val, float_val
            except (TypeError, ValueError):
                return None
    return None


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Load a juicer calibration CSV (from /mnt/analysis/data/juicer_calib/<name>.csv) "
            "and plot juice_level vs reward_mls with reward_number as a secondary x-axis. "
            "The plot is saved to /mnt/analysis/figures/juicer_calib/<name>.png."
        )
    )
    parser.add_argument(
        "name",
        help="Base filename (without extension) used for both the CSV input and PNG output.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display the plot after saving it (in addition to writing the PNG).",
    )
    args = parser.parse_args()

    csv_path = CSV_DIR / f"{args.name}.csv"
    png_path = FIG_DIR / f"{args.name}.png"
    png_path.parent.mkdir(parents=True, exist_ok=True)

    try:
        rows = load_csv_rows(csv_path)
    except FileNotFoundError:
        parser.error(f"File not found: {csv_path}")
    except ValueError as exc:
        parser.error(str(exc))

    print(f"Loaded {len(rows)} rows from {csv_path}")
    if rows:
        print("Columns:", ", ".join(rows[0].keys()))
        print("First row:", rows[0])

    reward_mls, reward_numbers, juice_levels_a4, juice_levels_a5 = extract_series(rows)

    fig, ax_left = plt.subplots(figsize=(10, 5))
    ax_right = ax_left.twinx()
    ax_top_left = ax_left.twiny()
    ax_top_right = ax_right.twiny()
    ax_top_right.spines["top"].set_position(("axes", 1.08))
    ax_top_right.spines["top"].set_visible(True)

    line_a4_mls, = ax_left.plot(
        reward_mls,
        juice_levels_a4,
        marker="o",
        linestyle="-",
        linewidth=1,
        markersize=3,
        color="tab:blue",
        label="A4 vs reward_mls",
    )
    line_a5_mls, = ax_right.plot(
        reward_mls,
        juice_levels_a5,
        marker="s",
        linestyle="-",
        linewidth=1,
        markersize=3,
        color="tab:green",
        label="A5 vs reward_mls",
    )

    ax_left.set_xlabel("reward_mls")
    ax_left.set_ylabel("juice_level (A4)", color="tab:blue")
    ax_right.set_ylabel("juice_level (A5)", color="tab:green")
    ax_left.set_title("Juice Levels (A4 & A5) vs reward_mls / reward_number")
    ax_left.grid(True, linestyle="--", alpha=0.4)

    line_a4_num, = ax_top_left.plot(
        reward_numbers,
        juice_levels_a4,
        color="tab:orange",
        linestyle="--",
        linewidth=1,
        alpha=0.8,
        label="A4 vs reward_number",
    )
    line_a5_num, = ax_top_right.plot(
        reward_numbers,
        juice_levels_a5,
        color="tab:red",
        linestyle="--",
        linewidth=1,
        alpha=0.8,
        label="A5 vs reward_number",
    )
    ax_top_left.set_xlabel("reward_number (A4)", color="tab:orange")
    ax_top_right.set_xlabel("reward_number (A5)", color="tab:red")

    lines = [line_a4_mls, line_a5_mls, line_a4_num, line_a5_num]
    labels = [line.get_label() for line in lines]
    ax_left.legend(lines, labels, loc="best")

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(png_path, dpi=150)
    print(f"Plot saved to {png_path}")

    if args.show:
        plt.show()
    else:
        plt.close(fig)


if __name__ == "__main__":
    main()


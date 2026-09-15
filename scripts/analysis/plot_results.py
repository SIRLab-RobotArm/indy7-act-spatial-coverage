#!/usr/bin/env python3
"""Draw the success-rate figure from the published rollout table.

    python scripts/analysis/plot_results.py
    python scripts/analysis/plot_results.py --split          # split by group
    python scripts/analysis/plot_results.py --input other.csv --output out.pdf

The default input is the 480-row result table that ships with this repository,
so the command works straight after cloning, with no robot and no model weights.

Reading the plot: each point is one training condition's success rate over its
120 evaluation trials, and the bars are Wilson 95% confidence intervals, which
stay honest at proportions near 0 and 1 where the normal approximation does not.

The figure is styled for print: black-and-white safe, vector output, and
TrueType fonts. Many conference submission systems reject Type 3 fonts, which
matplotlib embeds by default, so the `pdf.fonttype = 42` setting below is load
bearing - do not remove it.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import OrderedDict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT = (
    REPO_ROOT
    / "experiment/evaluation/main_recollection_20260817/results/rollouts.csv"
)
DEFAULT_OUTPUT = REPO_ROOT / "figures/success_rate.pdf"

matplotlib.rcParams.update({
    "pdf.fonttype": 42,
    "ps.fonttype": 42,
    "font.family": "serif",
    "font.size": 8,
    "axes.linewidth": 0.8,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
})

# Distinguish series by marker and line style, not by colour, so the figure
# survives greyscale printing.
STYLES = [
    dict(color="black", marker="o", linestyle="-", markerfacecolor="black"),
    dict(color="black", marker="s", linestyle="--", markerfacecolor="white"),
    dict(color="black", marker="^", linestyle="-.", markerfacecolor="0.6"),
]

GROUP_LABELS = {
    "exact_grid": "Trained positions",
    "q_unseen": "Unseen positions",
}


def wilson(successes: int, trials: int, z: float = 1.96) -> tuple[float, float, float]:
    """Wilson score interval for a binomial proportion."""
    if trials == 0:
        return 0.0, 0.0, 0.0
    p = successes / trials
    denominator = 1 + z**2 / trials
    centre = (p + z**2 / (2 * trials)) / denominator
    half = z * math.sqrt(
        p * (1 - p) / trials + z**2 / (4 * trials**2)) / denominator
    return p, max(0.0, centre - half), min(1.0, centre + half)


def load(path: Path, split: bool) -> "OrderedDict[str, OrderedDict[str, tuple[int, int]]]":
    """Aggregate (successes, trials) per condition, optionally per group."""
    series: OrderedDict[str, OrderedDict[str, tuple[int, int]]] = OrderedDict()
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            if row.get("analysis_eligible", "True").strip().lower() == "false":
                continue
            group = row.get("evaluation_group", "all") if split else "all"
            condition = row["condition"].strip()
            successes, trials = series.setdefault(
                group, OrderedDict()).get(condition, (0, 0))
            hit = int(row["success"].strip().lower() in {"true", "1"})
            series[group][condition] = (successes + hit, trials + 1)
    return series


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--split", action="store_true",
        help="draw trained and unseen positions as separate series")
    args = parser.parse_args()

    series = load(args.input, args.split)
    if not series:
        raise SystemExit(f"no usable rows in {args.input}")

    conditions = sorted({c for group in series.values() for c in group})
    figure, axes = plt.subplots(figsize=(4.6, 2.6))

    print(f"{'group':<20}{'condition':<11}{'successes':>11}{'rate':>9}{'95% CI':>16}")
    for index, (group, per_condition) in enumerate(series.items()):
        rates, low_error, high_error = [], [], []
        for condition in conditions:
            successes, trials = per_condition.get(condition, (0, 0))
            rate, low, high = wilson(successes, trials)
            rates.append(rate * 100)
            low_error.append((rate - low) * 100)
            high_error.append((high - rate) * 100)
            print(
                f"{GROUP_LABELS.get(group, group):<20}{condition:<11}"
                f"{successes:>5}/{trials:<5}{rate * 100:>8.1f}%"
                f"{low * 100:>8.1f}-{high * 100:.1f}"
            )
        axes.errorbar(
            range(len(conditions)), rates, yerr=[low_error, high_error],
            capsize=3, elinewidth=0.8, markersize=4,
            label=GROUP_LABELS.get(group, group),
            **STYLES[index % len(STYLES)])

    axes.set_xticks(range(len(conditions)))
    axes.set_xticklabels(conditions)
    axes.set_xlabel("Training condition")
    axes.set_ylabel("Success rate [%]")
    axes.set_ylim(0, 100)
    axes.grid(axis="y", color="0.85", linewidth=0.6)
    axes.set_axisbelow(True)
    for side in ("top", "right"):
        axes.spines[side].set_visible(False)
    if args.split:
        axes.legend(frameon=False, fontsize=7)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output)
    print(f"\nwrote {args.output}")


if __name__ == "__main__":
    main()

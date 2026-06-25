#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ---------------------------------------------------
# Paths
# ---------------------------------------------------

ROOT = Path.home() / "rgb-d_slam/icp_cpp"

SUMMARY = ROOT / "summary"

CSV_FILE = SUMMARY / "results.csv"

FIGURES = SUMMARY / "figures"

FIGURES.mkdir(parents=True, exist_ok=True)

# ---------------------------------------------------
# Read CSV
# ---------------------------------------------------

df = pd.read_csv(CSV_FILE)

# Convert to numeric
for col in [
    "ATE Mean",
    "ATE Median",
    "ATE Std",
    "ATE Max",
    "RPE Translation",
    "RPE Rotation"
]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

# Sort by Mean ATE (best first)

df = df.sort_values("ATE Mean")

# ---------------------------------------------------
# Save sorted CSV
# ---------------------------------------------------

sorted_csv = FIGURES / "summary_table.csv"

df.to_csv(sorted_csv, index=False)

# ---------------------------------------------------
# Comparison Figure
# ---------------------------------------------------

fig, ax = plt.subplots(3, 1, figsize=(10, 12))

# ---------------- Mean ----------------

bars = ax[0].bar(df["Scene"], df["ATE Mean"])

ax[0].set_title("Mean Absolute Trajectory Error")

ax[0].set_ylabel("ATE (m)")

ax[0].grid(axis="y", alpha=0.3)

for bar in bars:
    y = bar.get_height()
    ax[0].text(
        bar.get_x() + bar.get_width()/2,
        y + 0.03,
        f"{y:.2f}",
        ha="center",
        fontsize=9
    )

# ---------------- Std ----------------

bars = ax[1].bar(df["Scene"], df["ATE Std"])

ax[1].set_title("ATE Standard Deviation")

ax[1].set_ylabel("Std (m)")

ax[1].grid(axis="y", alpha=0.3)

for bar in bars:
    y = bar.get_height()
    ax[1].text(
        bar.get_x() + bar.get_width()/2,
        y + 0.02,
        f"{y:.2f}",
        ha="center",
        fontsize=9
    )

# ---------------- Max ----------------

bars = ax[2].bar(df["Scene"], df["ATE Max"])

ax[2].set_title("Maximum Absolute Trajectory Error")

ax[2].set_ylabel("Max Error (m)")

ax[2].grid(axis="y", alpha=0.3)

for bar in bars:
    y = bar.get_height()
    ax[2].text(
        bar.get_x() + bar.get_width()/2,
        y + 0.05,
        f"{y:.2f}",
        ha="center",
        fontsize=9
    )

for a in ax:
    a.tick_params(axis="x", rotation=30)

plt.tight_layout()

plt.savefig(
    FIGURES / "comparison_metrics.png",
    dpi=300,
    bbox_inches="tight"
)

plt.close()

# ---------------------------------------------------
# Summary Table Figure
# ---------------------------------------------------

fig, ax = plt.subplots(figsize=(9, 4))

ax.axis("off")

table = df[[
    "Scene",
    "ATE Mean",
    "ATE Std",
    "ATE Max"
]]

table = table.round(3)

tbl = ax.table(
    cellText=table.values,
    colLabels=table.columns,
    loc="center"
)

tbl.auto_set_font_size(False)

tbl.set_fontsize(10)

tbl.scale(1.2, 1.5)

plt.title(
    "Summary of ICP Results",
    fontsize=14,
    fontweight="bold",
    pad=20
)

plt.savefig(
    FIGURES / "summary_table.png",
    dpi=300,
    bbox_inches="tight"
)

plt.close()

# ---------------------------------------------------
# Print Best / Worst
# ---------------------------------------------------

best = df.iloc[0]
worst = df.iloc[-1]

print("="*60)

print("Comparison Figures Generated")

print("="*60)

print()

print("Best Scene")

print(
    f"{best['Scene']} "
    f"(ATE = {best['ATE Mean']:.3f} m)"
)

print()

print("Worst Scene")

print(
    f"{worst['Scene']} "
    f"(ATE = {worst['ATE Mean']:.3f} m)"
)

print()

print("Files saved in:")

print(FIGURES)

print()

print("Generated files:")

print("  comparison_metrics.png")

print("  summary_table.png")

print("  summary_table.csv")

print("="*60)
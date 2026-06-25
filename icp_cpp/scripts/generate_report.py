#!/usr/bin/env python3

import pandas as pd
import os

BASE = os.path.expanduser(
    "~/rgb-d_slam/icp_cpp/summary"
)

csv_file = os.path.join(BASE, "results.csv")

df = pd.read_csv(csv_file)

# Convert numeric columns
for col in df.columns[1:]:
    df[col] = pd.to_numeric(df[col], errors="coerce")

# Average metrics
with open(os.path.join(BASE, "average_metrics.txt"), "w") as f:

    f.write("Average ICP Results\n")
    f.write("=========================\n\n")

    for col in df.columns[1:]:

        f.write(
            f"{col}: {df[col].mean():.4f}\n"
        )

# Best Scene
best = df.loc[df["ATE Mean"].idxmin()]

with open(os.path.join(BASE, "best_scene.txt"), "w") as f:

    f.write(f"Best Scene\n\n")

    f.write(best.to_string())

# Worst Scene
worst = df.loc[df["ATE Mean"].idxmax()]

with open(os.path.join(BASE, "worst_scene.txt"), "w") as f:

    f.write(f"Worst Scene\n\n")

    f.write(worst.to_string())

print()
print("Report generated.")
print(BASE)
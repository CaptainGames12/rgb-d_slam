#!/usr/bin/env python3

import csv
import os
import re

BASE = os.path.expanduser(
    "~/rgb-d_slam/icp_cpp/results"
)

OUTPUT = os.path.expanduser(
    "~/rgb-d_slam/icp_cpp/summary/results.csv"
)

rows = []

for scene in sorted(os.listdir(BASE)):

    metrics = os.path.join(
        BASE,
        scene,
        "metrics.txt"
    )

    if not os.path.exists(metrics):
        continue

    with open(metrics) as f:
        text = f.read()

    def get(pattern):

        m = re.search(pattern, text)

        return m.group(1) if m else ""

    rows.append({

        "Scene": scene,

        "ATE Mean":
            get(r"Mean:\s+([0-9.]+) m"),

        "ATE Median":
            get(r"Median:\s+([0-9.]+) m"),

        "ATE Std":
            get(r"Std:\s+([0-9.]+) m"),

        "ATE Max":
            get(r"Max:\s+([0-9.]+) m"),

        "RPE Translation":
            get(r"Translation:\s+([0-9.]+)"),

        "RPE Rotation":
            get(r"Rotation:\s+([0-9.]+)")
    })

with open(OUTPUT, "w", newline="") as csvfile:

    writer = csv.DictWriter(
        csvfile,
        fieldnames=rows[0].keys()
    )

    writer.writeheader()

    writer.writerows(rows)

print()

print("Summary saved to")

print(OUTPUT)
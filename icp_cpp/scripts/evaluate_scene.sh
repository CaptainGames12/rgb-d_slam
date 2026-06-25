#!/bin/bash

SCENE=$1

if [ -z "$SCENE" ]; then
    echo "Usage:"
    echo "./evaluate_scene.sh scene0011_00"
    exit 1
fi

RESULT=~/rgb-d_slam/icp_cpp/results/$SCENE

mkdir -p "$RESULT"
mkdir -p "$RESULT/trajectories"
mkdir -p "$RESULT/plots"
mkdir -p "$RESULT/logs"

echo "========================================"
echo "Evaluating Scene : $SCENE"
echo "========================================"

# Remove old trajectory files
rm -f "$RESULT/trajectories/"*.txt

# Check source trajectory files
if [ ! -f ~/groundtruth_trajectory.txt ]; then
    echo "Ground truth trajectory not found!"
    exit 1
fi

if [ ! -f ~/estimated_trajectory.txt ]; then
    echo "Estimated trajectory not found!"
    exit 1
fi

# Copy trajectories
cp ~/groundtruth_trajectory.txt \
"$RESULT/trajectories/"

cp ~/estimated_trajectory.txt \
"$RESULT/trajectories/"

START=$(date +%s)

python3 \
~/rgb-d_slam/icp_cpp/scripts/evaluate_and_plot.py \
"$RESULT/trajectories/groundtruth_trajectory.txt" \
"$RESULT/trajectories/estimated_trajectory.txt" \
"$RESULT/plots" \
| tee "$RESULT/metrics.txt"

END=$(date +%s)

echo "" >> "$RESULT/run_info.txt"
echo "Evaluation Time : $((END-START)) sec" >> "$RESULT/run_info.txt"

cp "$RESULT/metrics.txt" "$RESULT/logs/evaluation.log"

echo
echo "========================================"
echo "Evaluation Complete"
echo "========================================"
echo "Results : $RESULT"
echo "========================================"
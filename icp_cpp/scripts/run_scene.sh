#!/bin/bash

#############################################
# ICP Evaluation Pipeline
#############################################

SCENE=$1

if [ -z "$SCENE" ]; then
    echo "Usage:"
    echo "./run_scene.sh scene0005_00"
    exit 1
fi

source ~/ros2_ws/install/setup.bash

DATASET=~/rgb-d_slam/datasets/$SCENE/$SCENE.sens

if [ ! -f "$DATASET" ]; then
    echo "Dataset not found:"
    echo "$DATASET"
    exit 1
fi

RESULT=~/rgb-d_slam/icp_cpp/results/$SCENE

mkdir -p "$RESULT"
mkdir -p "$RESULT/trajectories"
mkdir -p "$RESULT/plots"
mkdir -p "$RESULT/logs"

echo "==========================================="
echo "Scene : $SCENE"
echo "==========================================="

#############################################
# Clean old files
#############################################

rm -f ~/groundtruth_trajectory.txt
rm -f ~/estimated_trajectory.txt

#############################################
# Runtime
#############################################

START=$(date +%s)

#############################################
# Start ICP
#############################################

#############################################
# Cleanup old ROS nodes
#############################################

pkill -f "/home/swayam/ros2_ws/install/icp_cpp/lib/icp_cpp/icp_node" 2>/dev/null
pkill -f "save_groundtruth.py" 2>/dev/null
pkill -f "save_trajectory.py" 2>/dev/null

sleep 2

echo "Starting ICP..."

ros2 run icp_cpp icp_node \
> "$RESULT/logs/icp.log" 2>&1 &

ICP_PID=$!

sleep 2

#############################################
# Start Ground Truth Saver
#############################################

echo "Starting Ground Truth Saver..."

python3 \
~/rgb-d_slam/icp_cpp/scripts/save_groundtruth.py \
~/groundtruth_trajectory.txt \
> "$RESULT/logs/groundtruth.log" 2>&1 &

GT_PID=$!

#############################################
# Start Estimated Saver
#############################################

echo "Starting Estimated Saver..."

python3 \
~/rgb-d_slam/icp_cpp/scripts/save_trajectory.py \
~/estimated_trajectory.txt \
> "$RESULT/logs/estimated.log" 2>&1 &

EST_PID=$!

sleep 2

#############################################
# Run Publisher
#############################################

echo "Starting Publisher..."

ros2 run scannet_publisher scannet_publisher \
--ros-args \
-p file:=$DATASET \
| tee "$RESULT/logs/publisher.log"

echo ""
echo "Publisher Finished"

#############################################
# Wait for callbacks
#############################################

sleep 3

#############################################
# Stop Remaining Nodes
#############################################

# kill $ICP_PID
# kill $GT_PID
# kill $EST_PID
# kill -INT $ICP_PID 2>/dev/null
# kill -INT $GT_PID 2>/dev/null
# kill -INT $EST_PID 2>/dev/null

# wait $ICP_PID 2>/dev/null
# wait $GT_PID 2>/dev/null
# wait $EST_PID 2>/dev/null
kill -INT $ICP_PID 2>/dev/null
kill -INT $GT_PID 2>/dev/null
kill -INT $EST_PID 2>/dev/null

sleep 2

pkill -f "/home/swayam/ros2_ws/install/icp_cpp/lib/icp_cpp/icp_node" 2>/dev/null
pkill -f "save_groundtruth.py" 2>/dev/null
pkill -f "save_trajectory.py" 2>/dev/null

sleep 1

sleep 2

#############################################
# Runtime
#############################################

END=$(date +%s)

echo "" >> "$RESULT/run_info.txt"
echo "Runtime : $((END-START)) sec" >> "$RESULT/run_info.txt"

#############################################
# Evaluation
#############################################

echo ""
echo "Running Evaluation..."

~/rgb-d_slam/icp_cpp/scripts/evaluate_scene.sh $SCENE

echo ""
echo "==========================================="
echo "Experiment Complete"
echo "==========================================="
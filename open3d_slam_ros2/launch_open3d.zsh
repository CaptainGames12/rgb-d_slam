 
#!/bin/bash

SCENES=(
    "scene0011_00"
    "scene0011_01"
    "scene0050_01"
    "scene0378_00"
    "scene0378_02"
    "scene0518_00"
)

BASE_DIR="/second_home/capgames/scannet_data/scans"

for SCENE in "${SCENES[@]}"; do

    FILE_PATH="${BASE_DIR}/${SCENE}/${SCENE}.sens"

    echo "========================================================="
    echo "Begin with: ${SCENE}"
    echo "Path: ${FILE_PATH}"
    echo "========================================================="


    if [ -f "$FILE_PATH" ]; then

        ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
    sequence_file:="$FILE_PATH" \
    output_gt_file:="${BASE_DIR}/${SCENE}/${SCENE}_gt.txt" \
    output_trajectory_file:="${BASE_DIR}/${SCENE}/${SCENE}_trajectory.txt" \
    evaluate:="true"
    else
        echo "File is not found"
    fi

    echo ""
    echo "Scene is done: ${SCENE}"
    echo "---------------------------------------------------------"
    echo ""
done

echo "Done"

 
#!/bin/bash

# Список твоїх сцен
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
    python3 evaluate_and_plot.py \
${SCENE}/${SCENE}_gt.txt \
${SCENE}/${SCENE}_trajectory.txt \
slam_results_${SCENE}/

done

echo "🎉 Усі сцени оброблено!"

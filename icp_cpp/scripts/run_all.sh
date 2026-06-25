#!/bin/bash

CONFIG=~/rgb-d_slam/icp_cpp/config/scenes.txt

SUCCESS=0
FAILED=0

while read scene
do
    [ -z "$scene" ] && continue

    echo ""
    echo "======================================="
    echo "Running $scene"
    echo "======================================="

    ./run_scene.sh "$scene"

    if [ $? -eq 0 ]; then
        echo "✅ $scene completed."
        SUCCESS=$((SUCCESS+1))
    else
        echo "❌ $scene failed."
        FAILED=$((FAILED+1))
    fi

done < "$CONFIG"

echo ""
echo "======================================="
echo "Finished!"
echo "======================================="
echo "Successful : $SUCCESS"
echo "Failed     : $FAILED"
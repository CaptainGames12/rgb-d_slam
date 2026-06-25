# RGB-D SLAM Quick Start Guide

## Setup

```bash
cd /second_home/capgames/rgb-d_slam
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

## Run SLAM on a Sequence

```bash
# Default: saves results to ./slam_results/ in current directory
ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
  sequence_file:="/home/captaingames/Документи/scannet/scans/scene0010_01/scene0010_01.sens"

# Or specify custom output directory
ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
  sequence_file:="/home/captaingames/Документи/scannet/scans/scene0010_01/scene0010_01.sens" \
  output_dir:="/home/captaingames/slam_results/scene0010_01"
```

## Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `sequence_file` | (required) | Full path to ScanNet `.sens` file |
| `output_dir` | `./slam_results` | Where to save trajectory results |
| `parameter_file` | `configuration.lua` | Lua config file (optional) |
| `depth_topic` | `/camera/depth/image_raw` | Depth image ROS topic |
| `camera_info_topic` | `/camera/depth/camera_info` | Camera info ROS topic |
| `use_sim_time` | `true` | Use simulated time from bag/publisher |

## Output Files

After SLAM completes, check the output directory:

```bash
ls -la ./slam_results/
# estimated_trajectory.txt  <- Your SLAM trajectory in TUM format
```

## Evaluate Trajectory (After SLAM)

Once you have a ground truth file and estimated trajectory:

```bash
# Run evaluation + plotting together
python3 evaluate_and_plot.py \
  ground_truth.txt \
  ./slam_results/estimated_trajectory.txt \
  ./slam_results/

# Or just evaluation
python3 evaluate_trajectory.py \
  ground_truth.txt \
  ./slam_results/estimated_trajectory.txt

# Or just plotting
python3 plot_trajectory.py \
  ground_truth.txt \
  ./slam_results/estimated_trajectory.txt \
  ./slam_results/
```

## View Results

```bash
# Open trajectory plots
eog ./slam_results/*.png

# Or check metrics
cat evaluation_output.txt
```

## Troubleshooting

### "Failed to load parameter file"
The default Lua config may not exist. Provide an explicit one:
```bash
parameter_file:="/path/to/your/config.lua"
```

### Node crashes
Check logs:
```bash
cat ~/.ros/log/latest/open3d_slam_node-*/stderr
```

### Slow processing
Performance depends on sequence size. Processing is ~5-8 Hz for odometry. This is normal.

## Project Structure

- `src/open3d_slam_core/` - Core SLAM algorithm
- `src/open3d_slam_ros2/` - ROS2 integration
  - `launch/open3d_slam_eval_launch.py` - Main launch file
  - `src/open3d_slam_node.cpp` - ROS node implementation
  - `open3d_slam_ros2/evaluate_trajectory.py` - ATE/RPE metrics
  - `open3d_slam_ros2/plot_trajectory.py` - Visualization plots
  - `open3d_slam_ros2/evaluate_and_plot.py` - Combined evaluation + plotting

## Advanced: Custom Parameters

Edit or create a Lua config:
```bash
cp src/open3d_slam_core/include/example_param/configuration.lua my_config.lua
# Edit my_config.lua
ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
  sequence_file:="..." \
  parameter_file:="$(pwd)/my_config.lua"
```

See `TUNING_AND_EVALUATION.md` for parameter tuning guidance.

## Files Reference

- **USAGE.md** (this file) - Quick start guide
- **PLOTTING_GUIDE.md** - Detailed plotting/evaluation guide
- **TUNING_AND_EVALUATION.md** - Parameter tuning strategies

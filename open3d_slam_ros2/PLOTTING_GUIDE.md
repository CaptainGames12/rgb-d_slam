# Trajectory Evaluation and Plotting Guide

## Overview

The evaluation and plotting pipeline allows you to:
1. **Evaluate** trajectory accuracy against ground truth (compute ATE/RPE metrics)
2. **Visualize** trajectories and errors with publication-quality plots
3. **Compare** multiple runs or parameter configurations
4. **Tune** parameters based on error analysis

## Files

### Python Scripts
- **`evaluate_trajectory.py`**: Computes ATE (Absolute Trajectory Error) and RPE (Relative Pose Error)
- **`plot_trajectory.py`**: Generates 6 visualization plots from trajectory files
- **`evaluate_and_plot.py`**: Convenience wrapper that runs evaluation + plotting in sequence

### Output Files
The scripts generate:
- **Trajectory files** (TUM format): `timestamp x y z qx qy qz qw` (8 columns)
- **Evaluation metrics** (stdout): Mean/median/std ATE, mean/median/std RPE
- **PNG plots**: Numbered 01-06 for easy identification

## Quick Start

### Option 1: Evaluate + Plot Together
```bash
python3 evaluate_and_plot.py ground_truth.txt estimated_trajectory.txt output_dir/
```

This will:
1. Load and align trajectories
2. Compute ATE/RPE metrics and print to console
3. Generate 6 plots in `output_dir/`

### Option 2: Evaluate Only
```bash
python3 evaluate_trajectory.py ground_truth.txt estimated_trajectory.txt
```

Outputs:
```
============== TRAJECTORY EVALUATION ==============
Ground Truth:       1234 poses
Estimated:          1234 poses

Absolute Trajectory Error (ATE):
  Mean:   0.0345 m
  Median: 0.0312 m
  Std:    0.0156 m

Relative Pose Error (RPE):
  Mean:   0.0234 m (per meter traveled)
  Median: 0.0198 m
  Std:    0.0089 m
```

### Option 3: Plot Only
```bash
python3 plot_trajectory.py ground_truth.txt estimated_trajectory.txt output_dir/
```

Generates plots without metrics evaluation.

## Output Plots Explained

### 01_trajectory_3d.png
- **3D visualization** of ground truth (blue solid line) vs estimated (red dashed line)
- Start/end markers show trajectory beginning and end
- **Use for:** Overall trajectory shape comparison
- **Look for:** Major deviations, loop closures, drift patterns

### 02_trajectory_2d.png
- **Four 2D views**: Top-down (XY), side (XZ), side (YZ), distance traveled
- **Use for:** Detailed analysis of drift in specific directions
- **Look for:** Cumulative drift, height estimation errors, turning errors

### 03_position_components.png
- **X, Y, Z positions over time** (three subplots)
- Ground truth solid line, estimated dashed line
- **Use for:** Identifying which axes have problems
- **Look for:** Phase lag (delay), scale errors, systematic bias

### 04_orientation_components.png
- **Roll, Pitch, Yaw angles over time** (three subplots)
- **Use for:** Understanding rotation estimation quality
- **Look for:** Drift in specific rotation axes, gimbal lock effects

### 05_ate_over_time.png
- **Top**: ATE error vs time (filled area under curve)
- **Bottom**: Histogram of ATE errors with mean/median markers
- **Use for:** Identifying when errors grow, distribution of errors
- **Look for:** Increasing error over time (drift), outliers, error spikes

### 06_error_vectors.png
- **3D visualization** with ground truth (blue), estimated (red), and error vectors between them
- Error vectors colored by magnitude (dark red = small, bright red = large)
- Downsampled to ~30 vectors for clarity
- **Use for:** Spatial error distribution
- **Look for:** Localized errors, error clustering

## Typical Workflows

### Workflow 1: Quick Parameter Tuning
```bash
# Run with current parameters
ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py ...

# Evaluate and plot results
python3 evaluate_and_plot.py ground_truth.txt estimated_trajectory.txt run1/

# Adjust parameters in config.lua, run again
python3 evaluate_and_plot.py ground_truth.txt estimated_trajectory.txt run2/

# Compare plots visually
eog run1/ run2/  # Eye of GNOME
```

### Workflow 2: Batch Evaluation of Multiple Configs
```bash
# Create config directory structure
mkdir configs
cp config_tight.lua configs/
cp config_loose.lua configs/

# Run evaluations
for config in configs/*.lua; do
  output_dir="${config%.lua}_results"
  python3 evaluate_and_plot.py \
    ground_truth.txt \
    estimated_${config%.lua}.txt \
    $output_dir/
done

# View all results
ls -la configs/*_results/
```

### Workflow 3: Detailed Error Analysis
```bash
# Generate all plots
python3 plot_trajectory.py gt.txt est.txt plots/

# Start with high-level overview
eog plots/01_trajectory_3d.png      # Overall shape
eog plots/02_trajectory_2d.png      # Directional drift

# Dig deeper based on initial observation
eog plots/03_position_components.png # Which axes have errors?
eog plots/04_orientation_components.png # Rotation issues?
eog plots/05_ate_over_time.png      # When does error accumulate?
eog plots/06_error_vectors.png      # Where spatially are errors?
```

## Trajectory File Format

Both ground truth and estimated trajectories must be in **TUM format**:
```
timestamp x y z qx qy qz qw
1234567890.123 0.001 0.002 0.003 0.000 0.000 0.707 0.707
1234567891.123 0.005 0.007 0.009 0.000 0.000 0.707 0.707
...
```

- **timestamp**: seconds since epoch (float)
- **x, y, z**: position in meters (float)
- **qx, qy, qz, qw**: quaternion rotation (float, normalized)
- Space-separated columns
- One pose per line
- Lines starting with `#` are ignored (comments)

### Generate TUM Trajectory
If your trajectory is in different format, convert it:

**From 4x4 matrices (numpy):**
```python
import numpy as np
from scipy.spatial.transform import Rotation as R

def save_tum_trajectory(poses_dict, filename):
    """poses_dict: {timestamp: 4x4_matrix}"""
    with open(filename, 'w') as f:
        for ts in sorted(poses_dict.keys()):
            T = poses_dict[ts]
            pos = T[:3, 3]
            rot = R.from_matrix(T[:3, :3])
            quat = rot.as_quat()  # [qx, qy, qz, qw]
            f.write(f"{ts} {pos[0]} {pos[1]} {pos[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n")

save_tum_trajectory(my_poses, 'trajectory.txt')
```

**From KITTI format (x y z qx qy qz qw per row, different structure):**
```python
def kitti_to_tum(kitti_file, output_file, start_timestamp=0.0):
    """Convert KITTI poses to TUM format."""
    timestamps = []
    with open(output_file, 'w') as out:
        with open(kitti_file, 'r') as f:
            for idx, line in enumerate(f):
                T = np.array(line.split(), dtype=float).reshape(3, 4)
                T = np.vstack([T, [0, 0, 0, 1]])
                pos = T[:3, 3]
                rot = R.from_matrix(T[:3, :3])
                quat = rot.as_quat()
                timestamp = start_timestamp + idx * 0.033  # 30 Hz
                out.write(f"{timestamp} {pos[0]} {pos[1]} {pos[2]} {quat[0]} {quat[1]} {quat[2]} {quat[3]}\n")

kitti_to_tum('poses.txt', 'trajectory.txt')
```

## Metric Interpretation

### Absolute Trajectory Error (ATE)
- **Definition**: Euclidean distance between aligned ground truth and estimated poses at each timestamp
- **Formula**: `ATE[i] = ||T_gt[i].position - T_est_aligned[i].position||`
- **Units**: meters
- **What it means**: 
  - < 0.01m: Excellent (submillimeter precision)
  - 0.01-0.05m: Very good (typical for high-precision indoor SLAM)
  - 0.05-0.2m: Good (typical for moderate configs)
  - > 0.2m: Poor (likely parameter issues or drift)
- **Reported values**: Mean, median, standard deviation

### Relative Pose Error (RPE)
- **Definition**: Error in relative motion between pose pairs
- **Computation**: For poses > 10cm apart, error normalized by distance traveled
- **Units**: unitless (meters error per meter traveled)
- **What it means**:
  - < 0.01: Excellent (local motion very accurate)
  - 0.01-0.05: Very good (smooth trajectories)
  - 0.05-0.2: Good (acceptable local accuracy)
  - > 0.2: Poor (incorrect motion model or drift)
- **Interpretation**: RPE=0.05 means 1% error per meter (5cm error per 100m)

## Troubleshooting

### "trajectory files not found"
```bash
# Check file paths are correct
ls -la ground_truth.txt
ls -la estimated_trajectory.txt

# Verify file format
head -2 ground_truth.txt
head -2 estimated_trajectory.txt
```

### Empty plots or single points
**Possible causes:**
- Trajectory only has 1-2 poses (need at least 10)
- Wrong file format (not TUM)
- Timestamps not matching between GT and estimated

**Solution:**
```bash
# Count poses
wc -l ground_truth.txt
# Should have > 100 poses

# Check first few lines
head -3 ground_truth.txt
# Should look like: "1234567890.123 x y z qx qy qz qw"
```

### Plots show huge errors / trajectories far apart
- **Verify alignment**: Check if first poses are very different
- **Check quaternion normalization**: Quaternions should have magnitude ~1.0
- **Verify timestamps**: Make sure timestamps are reasonable (not years apart)

### AttributeError: No module named 'scipy'
```bash
pip install scipy matplotlib numpy
```

## Command Reference

### Print only metrics
```bash
python3 evaluate_trajectory.py gt.txt est.txt 2>/dev/null | grep "Mean:\|Median:"
```

### Generate plots silently
```bash
python3 plot_trajectory.py gt.txt est.txt output/ > /dev/null
```

### Compare multiple runs programmatically
```bash
#!/bin/bash
for run in run1 run2 run3; do
  echo "=== $run ==="
  python3 evaluate_trajectory.py \
    ground_truth.txt \
    "$run/trajectory.txt" 2>&1 | grep "Mean\|Median"
done
```

## Integration with ROS Launch

In your `open3d_slam_eval_launch.py`:

```python
# After SLAM completes
import subprocess
subprocess.run([
    'python3', 
    'evaluate_and_plot.py',
    'ground_truth.txt',
    'estimated_trajectory.txt',
    output_dir
])
```

This automatically generates plots after evaluation completes.

## Advanced: Custom Plotting

To modify plots (colors, line styles, fonts), edit `plot_trajectory.py`:

```python
# Line colors
ax.plot(..., 'b-', ...)          # 'b' = blue, '-' = solid line
                                  # 'r' = red, '--' = dashed, ':' = dotted

# Marker styles
ax.scatter(..., marker='o')       # 'o' = circle, 's' = square, '^' = triangle

# Font sizes
ax.set_xlabel('X (m)', fontsize=14)

# Save as PDF instead of PNG
plt.savefig(output_file.replace('.png', '.pdf'), dpi=300)
```

## See Also
- TUNING_AND_EVALUATION.md - Parameter tuning guide
- open3d_slam_eval_launch.py - ROS launch integration
- evaluate_trajectory.py - Metric computation details

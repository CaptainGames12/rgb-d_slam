# Open3D SLAM - Parameter Tuning & Trajectory Evaluation Guide

## Parameters to Reduce Trajectory Error

### High Impact (most important)

1. **Odometry - Scan Matching (per-scan accuracy)**
   ```lua
   odometry.scan_matching.cloud_registration_type = "PointToPlaneIcp"  -- vs GeneralizedIcp
   odometry.scan_matching.icp.max_correspondence_dist = 0.15  -- reduce from 1.0 (stricter)
   odometry.scan_processing.voxel_size = 0.05  -- reduce from 0.1 (more detail)
   odometry.scan_processing.downsampling_ratio = 0.2  -- reduce from 0.3 (keep more points)
   ```

2. **Mapper - Scan-to-Map Refinement**
   ```lua
   mapper_localizer.scan_to_map_registration.min_refinement_fitness = 0.85  -- increase from 0.7 (reject misaligned)
   mapper_localizer.scan_to_map_registration.icp.max_correspondence_dist = 0.15  -- reduce from 0.2
   ```

3. **Loop Closures - Prevent False Positives**
   ```lua
   mapper_localizer.is_attempt_loop_closures = true
   place_recognition.min_icp_refinement_fitness = 0.9  -- increase from 0.7 (stricter acceptance)
   place_recognition.ransac_probability = 0.999  -- increase from 0.99 (more robust)
   place_recognition.ransac_num_iter = 100000000  -- increase (more thorough)
   ```

### Medium Impact

4. **Odometry Buffer & Mapping Overlap**
   ```lua
   odometry.odometry_buffer_size = 5  -- increase (more history for refinement)
   submap.submaps_num_scan_overlap = 15  -- increase from 10 (smoother transitions)
   ```

5. **Motion Compensation (if sensor spins)**
   ```lua
   motion_compensation.is_undistort_scan = true  -- enable if spinning sensor
   motion_compensation.scan_duration = 0.1  -- match actual scan time
   ```

### Strategy by Scenario

**Indoor small room (known loop closure):**
- High voxel_size (0.3), lower refinement fitness (0.65)
- Strong loop closure: ransac_probability=0.999, max_icp_refinement_fitness=0.9

**Large outdoor (sparse/featureless):**
- Low voxel_size (0.05), high downsampling_ratio (0.9)
- Disable loop closures or use very high refinement threshold (0.95)

**Real-time performance:**
- Higher voxel_size (0.2), high downsampling_ratio (0.7)
- Disable dense map: is_build_dense_map=false

---

## Trajectory Evaluation Workflow

### 1. Launch SLAM with Evaluation

```bash
# Single sequence, use default parameters
ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
  sequence_file:=/path/to/sequence.bin \
  output_dir:=/tmp/slam_eval

# After SLAM finishes, evaluate:
python3 /path/to/evaluate_trajectory.py \
  /tmp/slam_eval/gt.txt \
  /tmp/slam_eval/estimated_trajectory.txt
```

### 2. Test Multiple Configurations

Create config variants in `include/example_param/`:
- `high_precision.lua` — low voxel_size, strict matching
- `fast.lua` — high voxel_size, loose matching
- `outdoor.lua` — disable loop closures

```bash
# Test config 1
ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
  sequence_file:=/data/seq1.bin \
  parameter_file:=/path/to/high_precision.lua \
  output_dir:=/tmp/test_hp

# Test config 2
ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
  sequence_file:=/data/seq1.bin \
  parameter_file:=/path/to/fast.lua \
  output_dir:=/tmp/test_fast
```

### 3. Evaluate Multiple Sequences

```bash
#!/bin/bash
sequences_dir=/data/scannet
configs=("high_precision.lua" "fast.lua")

for config in "${configs[@]}"; do
  for seq in $sequences_dir/*.bin; do
    name=$(basename "$seq" .bin)
    output="/tmp/eval_${config%.*}_${name}"
    ros2 launch open3d_slam_ros2 open3d_slam_eval_launch.py \
      sequence_file:="$seq" \
      parameter_file:="/path/to/$config" \
      output_dir:="$output"
    python3 evaluate_trajectory.py "$output/gt.txt" "$output/estimated_trajectory.txt"
  done
done
```

---

## Output Interpretation

### Metrics

- **ATE (Absolute Trajectory Error)** [meters]
  - Mean pose deviation from ground truth
  - Lower is better (< 0.1m is excellent)

- **RPE - Translation** [m/m]
  - Per-meter translation error
  - Lower is better (< 0.01 is excellent)

- **RPE - Rotation** [deg/m]
  - Per-meter rotation error
  - Lower is better (< 1° is excellent)

### Example Output

```
Absolute Trajectory Error (ATE):
  Mean:   0.0523 m
  Median: 0.0412 m
  Std:    0.0287 m
  Max:    0.1846 m

Relative Pose Error (RPE):
  Translation: 0.0089 m/m
  Rotation:    0.4231 deg/m
```

---

## Quick Tuning Checklist

- [ ] Check voxel_size < 0.1m for accuracy
- [ ] Check min_refinement_fitness > 0.75
- [ ] Check loop closure enabled (is_attempt_loop_closures=true)
- [ ] Check min_icp_refinement_fitness > 0.8 (prevent false loops)
- [ ] Run on known sequence with ground truth
- [ ] Compare ATE across configs
- [ ] Lock best config and test on other sequences

---

## Files

- `launch/open3d_slam_eval_launch.py` — Main launch with evaluation
- `open3d_slam_ros2/evaluate_trajectory.py` — Evaluation script (ATE/RPE computation)
- `include/example_param/configuration.lua` — Example config
- `include/example_param/default/default_parameters.lua` — Full parameter definitions

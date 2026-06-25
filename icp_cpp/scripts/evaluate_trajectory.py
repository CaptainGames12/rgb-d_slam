#!/usr/bin/env python3
"""
Evaluate trajectory by comparing ground truth (TUM format) with computed trajectory.
Computes Absolute Trajectory Error (ATE) and Relative Pose Error (RPE).
"""

import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path


def load_tum_trajectory(filepath):
    """
    Load trajectory in TUM format: timestamp x y z qx qy qz qw
    Returns: dict {timestamp: 4x4 transform matrix}
    """
    trajectory = {}
    with open(filepath, 'r') as f:
        for line in f:
            if line.startswith('#'):
                continue
            parts = line.strip().split()
            if len(parts) < 8:
                continue
            timestamp = float(parts[0])
            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
            
            # Build 4x4 transform
            rotation = R.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rotation
            T[:3, 3] = [x, y, z]
            trajectory[timestamp] = T
    return trajectory


def align_trajectories(gt_traj, est_traj):
    """
    Align estimated trajectory to ground truth using first pose.
    Returns: aligned estimated trajectory.
    """
    gt_timestamps = sorted(gt_traj.keys())
    est_timestamps = sorted(est_traj.keys())
    
    if not gt_timestamps or not est_timestamps:
        return {}
    
    # Use first common timestamp
    t0_gt = gt_timestamps[0]
    t0_est = est_timestamps[0]
    
    T_gt_0 = gt_traj[t0_gt]
    T_est_0 = est_traj[t0_est]
    
    # Alignment transform: T_aligned = T_gt_0 @ inv(T_est_0) @ T_est
    T_align = T_gt_0 @ np.linalg.inv(T_est_0)
    
    aligned = {}
    for ts, T_est in est_traj.items():
        aligned[ts] = T_align @ T_est
    return aligned


def compute_ate(gt_traj, est_traj):
    """
    Compute Absolute Trajectory Error (mean translation error).
    """
    if not gt_traj or not est_traj:
        return float('nan'), []
    
    # Find common timestamps (within tolerance)
    gt_ts = sorted(gt_traj.keys())
    est_ts = sorted(est_traj.keys())
    
    errors = []
    for ts_gt in gt_ts:
        # Find closest estimated timestamp
        ts_est = min(est_ts, key=lambda t: abs(t - ts_gt))
        if abs(ts_est - ts_gt) > 0.1:  # 100ms tolerance
            continue
        
        T_gt = gt_traj[ts_gt][:3, 3]
        T_est = est_traj[ts_est][:3, 3]
        error = np.linalg.norm(T_gt - T_est)
        errors.append(error)
    
    if not errors:
        return float('nan'), []
    return np.mean(errors), errors


def compute_rpe(gt_traj, est_traj):
    """
    Compute Relative Pose Error (mean rotation + translation error over 1m distance).
    """
    if not gt_traj or not est_traj:
        return float('nan'), float('nan')
    
    gt_ts = sorted(gt_traj.keys())
    est_ts = sorted(est_traj.keys())
    
    trans_errors = []
    rot_errors = []
    
    for i in range(len(gt_ts) - 1):
        t1_gt = gt_ts[i]
        t2_gt = gt_ts[i + 1]
        
        T1_gt = gt_traj[t1_gt]
        T2_gt = gt_traj[t2_gt]
        dT_gt = np.linalg.inv(T1_gt) @ T2_gt
        dist = np.linalg.norm(dT_gt[:3, 3])
        
        if dist < 0.1:  # Only consider pose pairs > 10cm apart
            continue
        
        # Find closest estimated timestamps
        ts1_est = min(est_ts, key=lambda t: abs(t - t1_gt))
        ts2_est = min(est_ts, key=lambda t: abs(t - t2_gt))
        
        if abs(ts1_est - t1_gt) > 0.1 or abs(ts2_est - t2_gt) > 0.1:
            continue
        
        T1_est = est_traj[ts1_est]
        T2_est = est_traj[ts2_est]
        dT_est = np.linalg.inv(T1_est) @ T2_est
        
        # Translation error
        trans_err = np.linalg.norm(dT_gt[:3, 3] - dT_est[:3, 3])
        trans_errors.append(trans_err / max(dist, 0.01))
        
        # Rotation error (angle)
        dR_error = dT_gt[:3, :3].T @ dT_est[:3, :3]
        angle = np.arccos(np.clip((np.trace(dR_error) - 1) / 2, -1, 1))
        rot_errors.append(np.degrees(angle))
    
    trans_rpe = np.mean(trans_errors) if trans_errors else float('nan')
    rot_rpe = np.mean(rot_errors) if rot_errors else float('nan')
    return trans_rpe, rot_rpe


def main(gt_file, est_file):
    """Evaluate trajectory."""
    gt_path = Path(gt_file)
    est_path = Path(est_file)
    
    if not gt_path.exists():
        print(f"Ground truth file not found: {gt_file}")
        return 1
    if not est_path.exists():
        print(f"Estimated trajectory file not found: {est_file}")
        return 1
    
    print(f"Loading ground truth: {gt_file}")
    gt_traj = load_tum_trajectory(gt_file)
    print(f"  Loaded {len(gt_traj)} poses")
    
    print(f"Loading estimated trajectory: {est_file}")
    est_traj = load_tum_trajectory(est_file)
    print(f"  Loaded {len(est_traj)} poses")
    
    # Align
    print("\nAligning trajectories...")
    est_traj_aligned = align_trajectories(gt_traj, est_traj)
    
    # Compute metrics
    print("\nComputing metrics...")
    ate_mean, ate_errors = compute_ate(gt_traj, est_traj_aligned)
    rpe_trans, rpe_rot = compute_rpe(gt_traj, est_traj_aligned)
    
    print("\n" + "="*60)
    print("TRAJECTORY EVALUATION RESULTS")
    print("="*60)
    print(f"Absolute Trajectory Error (ATE):")
    print(f"  Mean:   {ate_mean:.4f} m")
    if ate_errors:
        print(f"  Median: {np.median(ate_errors):.4f} m")
        print(f"  Std:    {np.std(ate_errors):.4f} m")
        print(f"  Max:    {np.max(ate_errors):.4f} m")
    print(f"\nRelative Pose Error (RPE):")
    print(f"  Translation: {rpe_trans:.4f} m/m")
    print(f"  Rotation:    {rpe_rot:.4f} deg/m")
    print("="*60)
    
    return 0


if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: evaluate_trajectory.py <gt_file> <est_file>")
        sys.exit(1)
    sys.exit(main(sys.argv[1], sys.argv[2]))

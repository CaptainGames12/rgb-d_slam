#!/usr/bin/env python3
"""
Plot trajectory evaluation results.
Generates trajectory comparison and error analysis plots.
"""

import sys
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_tum_trajectory(filepath):
    """Load trajectory in TUM format: timestamp x y z qx qy qz qw"""
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
            
            rotation = R.from_quat([qx, qy, qz, qw]).as_matrix()
            T = np.eye(4)
            T[:3, :3] = rotation
            T[:3, 3] = [x, y, z]
            trajectory[timestamp] = T
    return trajectory


def align_trajectories(gt_traj, est_traj):
    """Align estimated to ground truth using first pose."""
    gt_ts = sorted(gt_traj.keys())
    est_ts = sorted(est_traj.keys())
    
    if not gt_ts or not est_ts:
        return {}
    
    T_gt_0 = gt_traj[gt_ts[0]]
    T_est_0 = est_traj[est_ts[0]]
    T_align = T_gt_0 @ np.linalg.inv(T_est_0)
    
    aligned = {}
    for ts, T_est in est_traj.items():
        aligned[ts] = T_align @ T_est
    return aligned


def extract_positions_and_angles(trajectory):
    """Extract positions and Euler angles from trajectory."""
    timestamps = sorted(trajectory.keys())
    positions = []
    angles = []
    for ts in timestamps:
        pos = trajectory[ts][:3, 3]
        rot_mat = trajectory[ts][:3, :3]
        angle = R.from_matrix(rot_mat).as_euler('xyz', degrees=True)
        positions.append(pos)
        angles.append(angle)
    return np.array(timestamps), np.array(positions), np.array(angles)


def compute_ate_over_time(gt_traj, est_traj):
    """Compute ATE at each timestamp."""
    gt_ts = sorted(gt_traj.keys())
    est_ts = sorted(est_traj.keys())
    
    errors = []
    ts_list = []
    for ts_gt in gt_ts:
        ts_est = min(est_ts, key=lambda t: abs(t - ts_gt))
        if abs(ts_est - ts_gt) > 0.1:
            continue
        T_gt = gt_traj[ts_gt][:3, 3]
        T_est = est_traj[ts_est][:3, 3]
        error = np.linalg.norm(T_gt - T_est)
        errors.append(error)
        ts_list.append(ts_gt)
    
    return np.array(ts_list), np.array(errors)


def plot_trajectories_3d(gt_positions, est_positions, output_file=None):
    """Plot 3D trajectory comparison."""
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    # Ground truth
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], gt_positions[:, 2],
            'b-', linewidth=2, label='Ground Truth')
    ax.scatter(gt_positions[0, 0], gt_positions[0, 1], gt_positions[0, 2],
               c='blue', s=100, marker='o', label='GT Start')
    ax.scatter(gt_positions[-1, 0], gt_positions[-1, 1], gt_positions[-1, 2],
               c='blue', s=100, marker='s', label='GT End')
    
    # Estimated
    ax.plot(est_positions[:, 0], est_positions[:, 1], est_positions[:, 2],
            'r--', linewidth=2, label='Estimated')
    ax.scatter(est_positions[0, 0], est_positions[0, 1], est_positions[0, 2],
               c='red', s=100, marker='o', label='Est Start')
    ax.scatter(est_positions[-1, 0], est_positions[-1, 1], est_positions[-1, 2],
               c='red', s=100, marker='s', label='Est End')
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('3D Trajectory Comparison')
    ax.legend()
    ax.grid(True)
    
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
    return fig


def plot_trajectories_2d(gt_positions, est_positions, output_file=None):
    """Plot 2D trajectory comparison (top-down & side views)."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))
    
    # XY view (top-down)
    ax = axes[0, 0]
    ax.plot(gt_positions[:, 0], gt_positions[:, 1], 'b-', linewidth=2, label='Ground Truth')
    ax.plot(est_positions[:, 0], est_positions[:, 1], 'r--', linewidth=2, label='Estimated')
    ax.scatter(gt_positions[0, 0], gt_positions[0, 1], c='blue', s=100, marker='o', label='GT Start')
    ax.scatter(gt_positions[-1, 0], gt_positions[-1, 1], c='blue', s=100, marker='s', label='GT End')
    ax.scatter(est_positions[0, 0], est_positions[0, 1], c='red', s=100, marker='o')
    ax.scatter(est_positions[-1, 0], est_positions[-1, 1], c='red', s=100, marker='s')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Top-Down View (XY)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')
    
    # XZ view (side)
    ax = axes[0, 1]
    ax.plot(gt_positions[:, 0], gt_positions[:, 2], 'b-', linewidth=2, label='Ground Truth')
    ax.plot(est_positions[:, 0], est_positions[:, 2], 'r--', linewidth=2, label='Estimated')
    ax.scatter(gt_positions[0, 0], gt_positions[0, 2], c='blue', s=100, marker='o')
    ax.scatter(gt_positions[-1, 0], gt_positions[-1, 2], c='blue', s=100, marker='s')
    ax.scatter(est_positions[0, 0], est_positions[0, 2], c='red', s=100, marker='o')
    ax.scatter(est_positions[-1, 0], est_positions[-1, 2], c='red', s=100, marker='s')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Side View (XZ)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')
    
    # YZ view
    ax = axes[1, 0]
    ax.plot(gt_positions[:, 1], gt_positions[:, 2], 'b-', linewidth=2, label='Ground Truth')
    ax.plot(est_positions[:, 1], est_positions[:, 2], 'r--', linewidth=2, label='Estimated')
    ax.scatter(gt_positions[0, 1], gt_positions[0, 2], c='blue', s=100, marker='o')
    ax.scatter(gt_positions[-1, 1], gt_positions[-1, 2], c='blue', s=100, marker='s')
    ax.scatter(est_positions[0, 1], est_positions[0, 2], c='red', s=100, marker='o')
    ax.scatter(est_positions[-1, 1], est_positions[-1, 2], c='red', s=100, marker='s')
    ax.set_xlabel('Y (m)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Side View (YZ)')
    ax.legend()
    ax.grid(True)
    ax.axis('equal')
    
    # Distance from start
    ax = axes[1, 1]
    gt_dist = np.linalg.norm(gt_positions - gt_positions[0], axis=1)
    est_dist = np.linalg.norm(est_positions - est_positions[0], axis=1)
    t_gt = np.arange(len(gt_positions))
    t_est = np.arange(len(est_positions))
    ax.plot(t_gt, gt_dist, 'b-', linewidth=2, label='Ground Truth')
    ax.plot(t_est, est_dist, 'r--', linewidth=2, label='Estimated')
    ax.set_xlabel('Frame')
    ax.set_ylabel('Distance from Start (m)')
    ax.set_title('Distance Traveled')
    ax.legend()
    ax.grid(True)
    
    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
    return fig


def plot_ate_over_time(timestamps, ate_errors, output_file=None):
    """Plot ATE error over time."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # ATE over time
    ax1.plot(timestamps - timestamps[0], ate_errors, 'g-', linewidth=2)
    ax1.fill_between(timestamps - timestamps[0], ate_errors, alpha=0.3, color='green')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('ATE (m)')
    ax1.set_title('Absolute Trajectory Error Over Time')
    ax1.grid(True)
    
    # ATE statistics
    ax2.hist(ate_errors, bins=50, color='green', alpha=0.7, edgecolor='black')
    ax2.axvline(np.mean(ate_errors), color='red', linestyle='--', linewidth=2, label=f'Mean: {np.mean(ate_errors):.4f} m')
    ax2.axvline(np.median(ate_errors), color='orange', linestyle='--', linewidth=2, label=f'Median: {np.median(ate_errors):.4f} m')
    ax2.set_xlabel('ATE (m)')
    ax2.set_ylabel('Frequency')
    ax2.set_title('ATE Distribution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
    return fig


def plot_position_components(gt_timestamps, gt_positions, est_timestamps, est_positions, output_file=None):
    """Plot X, Y, Z position components separately."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    t_gt = gt_timestamps - gt_timestamps[0]
    t_est = est_timestamps - est_timestamps[0]
    
    for idx, (ax, label, component) in enumerate(zip(axes, ['X', 'Y', 'Z'], [0, 1, 2])):
        ax.plot(t_gt, gt_positions[:, component], 'b-', linewidth=2, label='Ground Truth')
        ax.plot(t_est, est_positions[:, component], 'r--', linewidth=2, label='Estimated')
        ax.set_ylabel(f'{label} (m)')
        ax.set_title(f'{label} Position Component')
        ax.legend()
        ax.grid(True)
    
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
    return fig


def plot_orientation_components(gt_timestamps, gt_angles, est_timestamps, est_angles, output_file=None):
    """Plot Roll, Pitch, Yaw separately."""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    
    t_gt = gt_timestamps - gt_timestamps[0]
    t_est = est_timestamps - est_timestamps[0]
    
    for idx, (ax, label, component) in enumerate(zip(axes, ['Roll', 'Pitch', 'Yaw'], [0, 1, 2])):
        ax.plot(t_gt, gt_angles[:, component], 'b-', linewidth=2, label='Ground Truth')
        ax.plot(t_est, est_angles[:, component], 'r--', linewidth=2, label='Estimated')
        ax.set_ylabel(f'{label} (deg)')
        ax.set_title(f'{label} Angle')
        ax.legend()
        ax.grid(True)
    
    axes[-1].set_xlabel('Time (s)')
    plt.tight_layout()
    if output_file:
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
    return fig


def plot_error_vectors(gt_traj, est_traj, output_file=None):
    """Plot error vectors at regular intervals matching by time."""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')

    gt_ts = sorted(gt_traj.keys())
    est_ts = sorted(est_traj.keys())

    # Extract arrays just for plotting the solid lines
    gt_pos_arr = np.array([gt_traj[t][:3, 3] for t in gt_ts])
    est_pos_arr = np.array([est_traj[t][:3, 3] for t in est_ts])

    ax.plot(gt_pos_arr[:, 0], gt_pos_arr[:, 1], gt_pos_arr[:, 2],
            'b-', linewidth=1, alpha=0.5, label='Ground Truth')
    ax.plot(est_pos_arr[:, 0], est_pos_arr[:, 1], est_pos_arr[:, 2],
            'r-', linewidth=1, alpha=0.5, label='Estimated')

    # Downsample for clarity
    step = max(1, len(gt_ts) // 30)
    indices = range(0, len(gt_ts), step)

    # Plot error vectors by matching time!
    for i in indices:
        ts_gt = gt_ts[i]
        # Find closest estimated time
        ts_est = min(est_ts, key=lambda t: abs(t - ts_gt))

        # Only plot vector if time difference is small
        if abs(ts_est - ts_gt) <= 0.5:
            gt_pos = gt_traj[ts_gt][:3, 3]
            est_pos = est_traj[ts_est][:3, 3]

            # Draw line between them
            ax.plot([gt_pos[0], est_pos[0]],
                    [gt_pos[1], est_pos[1]],
                    [gt_pos[2], est_pos[2]], 'k-', alpha=0.5)

    ax.set_title('Trajectory Error Vectors')
    ax.legend()

    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
    plt.close(fig)


def main(gt_file, est_file, output_dir=None):
    """Generate all plots."""
    gt_path = Path(gt_file)
    est_path = Path(est_file)
    
    if not gt_path.exists() or not est_path.exists():
        print(f"Error: trajectory files not found")
        return 1
    
    if output_dir is None:
        output_dir = gt_path.parent
    else:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Loading trajectories...")
    gt_traj = load_tum_trajectory(gt_file)
    est_traj = load_tum_trajectory(est_file)
    
    print(f"Aligning trajectories...")
    est_traj_aligned = align_trajectories(gt_traj, est_traj)
    
    # Extract data
    gt_ts, gt_pos, gt_ang = extract_positions_and_angles(gt_traj)
    est_ts, est_pos, est_ang = extract_positions_and_angles(est_traj_aligned)
    ate_ts, ate_errors = compute_ate_over_time(gt_traj, est_traj_aligned)
    
    print(f"Generating plots...")
    
    # 3D trajectory
    plot_trajectories_3d(gt_pos, est_pos, 
                         output_dir / '01_trajectory_3d.png')
    
    # 2D views
    plot_trajectories_2d(gt_pos, est_pos,
                        output_dir / '02_trajectory_2d.png')
    
    # Position components
    plot_position_components(gt_ts, gt_pos, est_ts, est_pos,
                            output_dir / '03_position_components.png')
    
    # Orientation components
    plot_orientation_components(gt_ts, gt_ang, est_ts, est_ang,
                               output_dir / '04_orientation_components.png')
    
    # ATE over time
    plot_ate_over_time(ate_ts, ate_errors,
                      output_dir / '05_ate_over_time.png')
    
    # Error vectors
    plot_error_vectors(gt_traj, est_traj_aligned,
                      output_dir / '06_error_vectors.png')
    
    print(f"\nPlots saved to: {output_dir}")
    print(f"  01_trajectory_3d.png")
    print(f"  02_trajectory_2d.png")
    print(f"  03_position_components.png")
    print(f"  04_orientation_components.png")
    print(f"  05_ate_over_time.png")
    print(f"  06_error_vectors.png")
    
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: plot_trajectory.py <gt_file> <est_file> [output_dir]")
        sys.exit(1)
    
    output_dir = sys.argv[3] if len(sys.argv) > 3 else None
    sys.exit(main(sys.argv[1], sys.argv[2], output_dir))

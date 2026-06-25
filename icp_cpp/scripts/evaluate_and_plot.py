#!/usr/bin/env python3
"""
Evaluate trajectory and generate plots in one command.
Usage: evaluate_and_plot.py <gt_file> <est_file> [output_dir]
"""

import sys
import subprocess
from pathlib import Path


def run_evaluation(gt_file, est_file, output_dir):
    """Run evaluation script."""
    print("=" * 60)
    print("EVALUATING TRAJECTORY")
    print("=" * 60)
    
    eval_script = Path(__file__).parent / 'evaluate_trajectory.py'
    result = subprocess.run([
        sys.executable, str(eval_script), gt_file, est_file
    ], capture_output=False)
    
    return result.returncode == 0


def run_plotting(gt_file, est_file, output_dir):
    """Run plotting script."""
    print("\n" + "=" * 60)
    print("GENERATING PLOTS")
    print("=" * 60 + "\n")
    
    plot_script = Path(__file__).parent / 'plot_trajectory.py'
    result = subprocess.run([
        sys.executable, str(plot_script), gt_file, est_file, output_dir
    ], capture_output=False)
    
    return result.returncode == 0


def main(gt_file, est_file, output_dir=None):
    """Evaluate and plot."""
    gt_path = Path(gt_file)
    est_path = Path(est_file)
    
    if not gt_path.exists():
        print(f"Error: Ground truth file not found: {gt_file}")
        return 1
    
    if not est_path.exists():
        print(f"Error: Estimated trajectory file not found: {est_file}")
        return 1
    
    if output_dir is None:
        output_dir = str(gt_path.parent)
    
    # Run evaluation
    if not run_evaluation(gt_file, est_file, output_dir):
        print("Evaluation failed")
        return 1
    
    # Run plotting
    if not run_plotting(gt_file, est_file, output_dir):
        print("Plotting failed")
        return 1
    
    print("\n" + "=" * 60)
    print("DONE!")
    print("=" * 60)
    print(f"\nPlots saved to: {output_dir}")
    print("Open plots with:")
    print(f"  eog {output_dir}/*.png  # Eye of GNOME")
    print(f"  feh {output_dir}/*.png  # Feh")
    print(f"  or view in file manager")
    
    return 0


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: evaluate_and_plot.py <gt_file> <est_file> [output_dir]")
        sys.exit(1)
    
    output_dir = sys.argv[3] if len(sys.argv) > 3 else None
    sys.exit(main(sys.argv[1], sys.argv[2], output_dir))

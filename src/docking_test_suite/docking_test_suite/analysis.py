"""Post-test analysis and visualization for the docking test suite.

Reads CSV files produced by the test scripts and generates:
- Error statistics (mean, std, max, RMSE per label)
- Error vs. distance plot (Test A)
- Error vs. angle plot (Test B)
- Error over time plot (Test C / D)
- 3-D trajectory comparison (ground truth vs. camera estimate)
- Pose-jump detection (for dynamic tests)
- Cross-condition comparison bar chart (Test D)
- Summary report printed to stdout
"""

import os
import sys
import argparse

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend (works on headless Jetson)
import matplotlib.pyplot as plt


# ------------------------------------------------------------------
# Data loading
# ------------------------------------------------------------------

def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Convert NaN-string columns
    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors='ignore')
    return df


# ------------------------------------------------------------------
# Statistics
# ------------------------------------------------------------------

def compute_stats(df: pd.DataFrame) -> pd.DataFrame:
    """Per-label error statistics."""
    grouped = df.groupby('test_label')
    rows = []
    for label, g in grouped:
        eu = g['error_euclidean'].dropna()
        if eu.empty:
            continue
        rows.append({
            'label': label,
            'count': len(eu),
            'mean_err': eu.mean(),
            'std_err': eu.std(),
            'max_err': eu.max(),
            'min_err': eu.min(),
            'rmse': np.sqrt((eu ** 2).mean()),
            'mean_err_x': g['error_x'].dropna().mean(),
            'mean_err_y': g['error_y'].dropna().mean(),
            'mean_err_z': g['error_z'].dropna().mean(),
        })
    return pd.DataFrame(rows)


def detect_pose_jumps(df: pd.DataFrame, threshold: float = 0.1) -> pd.DataFrame:
    """Detect discontinuities in the camera-estimated position.

    A "jump" is when the Euclidean distance between consecutive frames
    exceeds *threshold* metres.
    """
    cx = df['cam_rel_x'].values
    cy = df['cam_rel_y'].values
    cz = df['cam_rel_z'].values

    jumps = []
    for i in range(1, len(df)):
        dx = cx[i] - cx[i - 1]
        dy = cy[i] - cy[i - 1]
        dz = cz[i] - cz[i - 1]
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        if dist > threshold:
            jumps.append({
                'index': i,
                'elapsed_s': df['elapsed_s'].iloc[i],
                'jump_m': dist,
                'cam_x': cx[i], 'cam_y': cy[i], 'cam_z': cz[i],
            })
    return pd.DataFrame(jumps) if jumps else pd.DataFrame()


# ------------------------------------------------------------------
# Plotting helpers
# ------------------------------------------------------------------

def _save_fig(fig, out_dir, name):
    path = os.path.join(out_dir, f'{name}.png')
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f'  Saved: {path}')


def plot_error_vs_distance(df: pd.DataFrame, out_dir: str):
    """Test A: error as a function of nominal distance."""
    stats = compute_stats(df)
    if stats.empty:
        return

    # Extract distance from label "distance_X.Ym"
    stats['distance'] = stats['label'].str.extract(r'distance_([\d.]+)m').astype(float)
    stats = stats.dropna(subset=['distance']).sort_values('distance')

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.errorbar(stats['distance'], stats['mean_err'],
                yerr=stats['std_err'], fmt='o-', capsize=4, label='Mean +/- Std')
    ax.plot(stats['distance'], stats['max_err'], 'r^--', label='Max Error')
    ax.set_xlabel('Distance (m)')
    ax.set_ylabel('Euclidean Error (m)')
    ax.set_title('Test A: Error vs. Distance')
    ax.legend()
    ax.grid(True, alpha=0.3)
    _save_fig(fig, out_dir, 'test_a_error_vs_distance')


def plot_error_vs_angle(df: pd.DataFrame, out_dir: str):
    """Test B: error as a function of nominal angle."""
    stats = compute_stats(df)
    if stats.empty:
        return

    stats['angle'] = stats['label'].str.extract(r'angle_([+-]?\d+)deg').astype(float)
    stats = stats.dropna(subset=['angle']).sort_values('angle')

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.errorbar(stats['angle'], stats['mean_err'],
                yerr=stats['std_err'], fmt='o-', capsize=4, label='Mean +/- Std')
    ax.plot(stats['angle'], stats['max_err'], 'r^--', label='Max Error')
    ax.set_xlabel('Angle (degrees)')
    ax.set_ylabel('Euclidean Error (m)')
    ax.set_title('Test B: Error vs. Viewing Angle')
    ax.legend()
    ax.grid(True, alpha=0.3)
    _save_fig(fig, out_dir, 'test_b_error_vs_angle')


def plot_error_over_time(df: pd.DataFrame, out_dir: str, name_prefix: str = 'test_c'):
    """Test C/D: error as a function of elapsed time."""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    t = df['elapsed_s'].values
    eu = df['error_euclidean'].values
    cx = df['cam_rel_x'].values
    cy = df['cam_rel_y'].values
    cz = df['cam_rel_z'].values

    # Error over time
    ax1.plot(t, eu, '.', markersize=2, alpha=0.6)
    ax1.set_ylabel('Euclidean Error (m)')
    ax1.set_title(f'{name_prefix.upper()}: Error During Approach')
    ax1.grid(True, alpha=0.3)

    # Pose jumps
    jumps = detect_pose_jumps(df)
    if not jumps.empty:
        ax1.axhline(y=0.1, color='r', linestyle='--', alpha=0.5, label='Jump threshold')
        for _, j in jumps.iterrows():
            ax1.axvline(x=j['elapsed_s'], color='r', alpha=0.3, linewidth=0.5)
        ax1.legend()

    # Camera-estimated distance over time
    cam_dist = np.sqrt(cx**2 + cy**2 + cz**2)
    ax2.plot(t, cam_dist, '.', markersize=2, alpha=0.6, color='green')
    ax2.set_xlabel('Elapsed Time (s)')
    ax2.set_ylabel('Camera Estimated Distance (m)')
    ax2.grid(True, alpha=0.3)

    _save_fig(fig, out_dir, f'{name_prefix}_error_over_time')


def plot_trajectory_comparison(df: pd.DataFrame, out_dir: str,
                               name_prefix: str = 'test_c'):
    """2-D trajectory comparison: ground truth vs camera estimate (XY plane)."""
    fig, ax = plt.subplots(figsize=(8, 8))

    gt_valid = df[['gt_rel_x', 'gt_rel_y']].dropna()
    cam_valid = df[['cam_rel_x', 'cam_rel_y']].dropna()

    if not gt_valid.empty:
        ax.plot(gt_valid['gt_rel_x'], gt_valid['gt_rel_y'],
                '.-', markersize=3, alpha=0.6, label='Vicon (ground truth)')
    if not cam_valid.empty:
        ax.plot(cam_valid['cam_rel_x'], cam_valid['cam_rel_y'],
                '.-', markersize=3, alpha=0.6, label='Camera (solvePnP)')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title(f'{name_prefix.upper()}: Trajectory Comparison (XY)')
    ax.legend()
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    _save_fig(fig, out_dir, f'{name_prefix}_trajectory_xy')


def plot_condition_comparison(csv_paths: list, out_dir: str):
    """Test D: bar chart comparing mean error across conditions."""
    labels = []
    means = []
    stds = []

    for path in csv_paths:
        df = load_csv(path)
        eu = df['error_euclidean'].dropna()
        if eu.empty:
            continue
        name = df['test_name'].iloc[0] if 'test_name' in df else os.path.basename(path)
        # Extract condition from test name
        cond = name.replace('test_d_', '').replace('test_c_dynamic', 'baseline')
        labels.append(cond)
        means.append(eu.mean())
        stds.append(eu.std())

    if not labels:
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(labels))
    ax.bar(x, means, yerr=stds, capsize=5, alpha=0.8, color='steelblue')
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha='right')
    ax.set_ylabel('Mean Euclidean Error (m)')
    ax.set_title('Test D: Error Across Environmental Conditions')
    ax.grid(True, alpha=0.3, axis='y')
    _save_fig(fig, out_dir, 'test_d_condition_comparison')


def plot_error_heatmap(df: pd.DataFrame, out_dir: str, name_prefix: str = ''):
    """XY error heatmap."""
    ex = df['error_x'].dropna().values
    ey = df['error_y'].dropna().values
    if len(ex) < 2:
        return

    fig, ax = plt.subplots(figsize=(7, 6))
    h = ax.hist2d(ex, ey, bins=30, cmap='hot')
    fig.colorbar(h[3], ax=ax, label='Count')
    ax.set_xlabel('Error X (m)')
    ax.set_ylabel('Error Y (m)')
    ax.set_title(f'{name_prefix} Error Heatmap (XY)')
    ax.set_aspect('equal')
    _save_fig(fig, out_dir, f'{name_prefix}_error_heatmap')


def plot_component_errors(df: pd.DataFrame, out_dir: str, name_prefix: str = ''):
    """Per-axis error over time."""
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
    t = df['elapsed_s'].values

    for ax, col, label in zip(
        axes,
        ['error_x', 'error_y', 'error_z'],
        ['Error X', 'Error Y', 'Error Z'],
    ):
        vals = df[col].values
        ax.plot(t, vals, '.', markersize=2, alpha=0.6)
        ax.set_ylabel(f'{label} (m)')
        ax.grid(True, alpha=0.3)
        ax.axhline(0, color='k', linewidth=0.5)

    axes[0].set_title(f'{name_prefix} Per-Axis Error Over Time')
    axes[-1].set_xlabel('Elapsed Time (s)')
    _save_fig(fig, out_dir, f'{name_prefix}_component_errors')


# ------------------------------------------------------------------
# Summary report
# ------------------------------------------------------------------

def print_summary(df: pd.DataFrame, csv_path: str):
    """Print a human-readable summary to stdout."""
    stats = compute_stats(df)
    test_name = df['test_name'].iloc[0] if 'test_name' in df and len(df) > 0 else 'unknown'

    print(f'\n{"="*60}')
    print(f'  ANALYSIS SUMMARY: {test_name}')
    print(f'  Source: {csv_path}')
    print(f'  Total samples: {len(df)}')
    print(f'{"="*60}')

    if stats.empty:
        print('  No valid error data found.')
        return

    # Overall
    eu = df['error_euclidean'].dropna()
    print(f'\n  Overall Euclidean Error:')
    print(f'    Mean:  {eu.mean():.4f} m')
    print(f'    Std:   {eu.std():.4f} m')
    print(f'    Max:   {eu.max():.4f} m')
    print(f'    RMSE:  {np.sqrt((eu**2).mean()):.4f} m')

    # Per-label
    print(f'\n  Per-Label Breakdown:')
    print(f'  {"Label":<25s} {"Count":>6s} {"Mean":>8s} {"Std":>8s} '
          f'{"Max":>8s} {"RMSE":>8s}')
    print(f'  {"-"*73}')
    for _, row in stats.iterrows():
        print(f'  {row["label"]:<25s} {row["count"]:>6.0f} '
              f'{row["mean_err"]:>8.4f} {row["std_err"]:>8.4f} '
              f'{row["max_err"]:>8.4f} {row["rmse"]:>8.4f}')

    # Pose jumps (for dynamic tests)
    jumps = detect_pose_jumps(df)
    if not jumps.empty:
        print(f'\n  Pose Jumps Detected: {len(jumps)}')
        for _, j in jumps.iterrows():
            print(f'    t={j["elapsed_s"]:.2f}s  jump={j["jump_m"]:.4f}m')


# ------------------------------------------------------------------
# Main analysis pipeline
# ------------------------------------------------------------------

def analyze_csv(csv_path: str, out_dir: str = None):
    """Run full analysis on a single CSV file."""
    csv_path = os.path.expanduser(csv_path)
    if not os.path.isfile(csv_path):
        print(f'File not found: {csv_path}')
        return

    if out_dir is None:
        out_dir = os.path.join(os.path.dirname(csv_path), 'analysis')
    os.makedirs(out_dir, exist_ok=True)

    df = load_csv(csv_path)
    test_name = df['test_name'].iloc[0] if 'test_name' in df and len(df) > 0 else ''

    print_summary(df, csv_path)

    # Generate appropriate plots based on test type
    if 'test_a' in test_name:
        plot_error_vs_distance(df, out_dir)
        plot_error_heatmap(df, out_dir, 'test_a')
    elif 'test_b' in test_name:
        plot_error_vs_angle(df, out_dir)
        plot_error_heatmap(df, out_dir, 'test_b')
    elif 'test_c' in test_name or 'test_d' in test_name:
        prefix = test_name if test_name else 'dynamic'
        plot_error_over_time(df, out_dir, prefix)
        plot_trajectory_comparison(df, out_dir, prefix)
        plot_component_errors(df, out_dir, prefix)
        plot_error_heatmap(df, out_dir, prefix)

    # Always generate component errors if we have time data
    if 'test_a' in test_name or 'test_b' in test_name:
        plot_component_errors(df, out_dir, test_name)

    print(f'\nPlots saved to: {out_dir}')


def main():
    parser = argparse.ArgumentParser(
        description='Analyze docking test suite CSV data')
    parser.add_argument('csv_files', nargs='+', help='CSV file(s) to analyze')
    parser.add_argument('--output', '-o', default=None,
                        help='Output directory for plots')
    parser.add_argument('--compare', action='store_true',
                        help='Generate cross-condition comparison (Test D)')
    args = parser.parse_args()

    if args.compare and len(args.csv_files) > 1:
        out = args.output or os.path.join(os.path.dirname(args.csv_files[0]), 'analysis')
        os.makedirs(out, exist_ok=True)
        plot_condition_comparison(args.csv_files, out)

    for csv_path in args.csv_files:
        analyze_csv(csv_path, args.output)


if __name__ == '__main__':
    main()

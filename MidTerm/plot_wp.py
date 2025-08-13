# plot_trajectory.py
from __future__ import division
import sys, os, argparse
import numpy as np
import pandas as pd

import matplotlib
matplotlib.use('Agg')  # non-GUI backend
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

RING_RADIUS = 400.0  # mm

def read_waypoints(path):
    pts = []
    with open(path, 'r') as f:
        for raw in f:
            s = raw.strip()
            if not s or s.startswith('#'):
                continue
            parts = [p.strip() for p in s.split(',')]
            if len(parts) >= 2:
                try:
                    pts.append([float(parts[0]), float(parts[1])])
                except:
                    pass
    if len(pts) < 2:
        raise ValueError("Waypoint list must contain at least 2 points.")
    return np.asarray(pts, dtype=float)

# Example usage:
#   python plot_trajectory.py waypoint_list.txt run_log.csv
#
# Produces:
#   traj_<csvbase>.png
#   cte_<csvbase>.png
#   heading_<csvbase>.png

def main():
    ap = argparse.ArgumentParser(description="Plot XY trajectory, CTE, and heading error.")
    ap.add_argument("waypoints", help="waypoint_list.txt")
    ap.add_argument("logcsv", help="CSV log with at least x_mm,y_mm (and optionally t_sec, cte_mm, head_err_deg)")
    args = ap.parse_args()

    # Load
    wp = read_waypoints(args.waypoints)
    df = pd.read_csv(args.logcsv)

    base = os.path.splitext(os.path.basename(args.logcsv))[0]
    out_traj = os.path.abspath("traj_%s.png" % base)
    out_cte = os.path.abspath("cte_%s.png" % base)
    out_head = os.path.abspath("heading_%s.png" % base)

    # --- Trajectory ---
    if "x_mm" not in df.columns or "y_mm" not in df.columns:
        print("CSV must contain columns named x_mm and y_mm.")
        print("Found columns:", list(df.columns))
        sys.exit(1)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(df["x_mm"], df["y_mm"], linewidth=1.5, label="Trajectory")
    ax.plot(wp[:, 0], wp[:, 1], 'o', ms=5, label="Waypoints")
    for i, (wx, wy) in enumerate(wp):
        ax.text(wx, wy, str(i + 1), fontsize=8, va='bottom', ha='left')
        ax.add_patch(Circle((wx, wy), RING_RADIUS, fill=False, lw=1.0, alpha=0.7))
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True, linestyle='--', alpha=0.4)
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_title("Trajectory vs Waypoints (+400 mm rings)")
    ax.legend(loc='best')
    fig.tight_layout()
    fig.savefig(out_traj, dpi=150)
    plt.close(fig)
    print("Saved:", out_traj)

    # --- CTE plot ---
    if all(c in df.columns for c in ["t_sec", "cte_mm"]):
        fig2, ax2 = plt.subplots(figsize=(10, 5))
        ax2.plot(df["t_sec"], df["cte_mm"], linewidth=1.5, label="CTE (mm)")
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("CTE (mm)")
        ax2.grid(True, linestyle='--', alpha=0.4)
        ax2.set_title("Cross-Track Error Over Time")
        fig2.tight_layout()
        fig2.savefig(out_cte, dpi=150)
        plt.close(fig2)
        print("Saved:", out_cte)
    else:
        print("Skipping CTE plot (need columns: t_sec, cte_mm).")

    # --- Heading error plot ---
    if all(c in df.columns for c in ["t_sec", "head_err_deg"]):
        fig3, ax3 = plt.subplots(figsize=(10, 5))
        ax3.plot(df["t_sec"], df["head_err_deg"], linewidth=1.5, label="Heading Error (deg)")
        ax3.set_xlabel("Time (s)")
        ax3.set_ylabel("Heading Error (deg)")
        ax3.grid(True, linestyle='--', alpha=0.4)
        ax3.set_title("Heading Error Over Time")
        fig3.tight_layout()
        fig3.savefig(out_head, dpi=150)
        plt.close(fig3)
        print("Saved:", out_head)
    else:
        print("Skipping heading error plot (need columns: t_sec, head_err_deg).")

if __name__ == "__main__":
    main()

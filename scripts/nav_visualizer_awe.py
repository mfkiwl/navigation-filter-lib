#!/usr/bin/env python3
"""
nav_visualizer_new.py — Visualizer for the new dataset (truth.nav / GNSS_RTK.pos)

Inputs:
- Navigation results (SaveResults): columns =
  [index, lat(deg), lon(deg), h(m), vE(m/s), vN(m/s), vU(m/s), yaw(deg), pitch(deg), roll(deg)]
- Truth file (truth.nav): columns =
  [Week, SOW, Lat(deg), Lon(deg), H(m), vN, vE, vD, Roll(deg), Pitch(deg), Yaw(deg)]
  Internally we use: vE=vE, vN=vN, vU=-vD; Heading = 360 - Yaw(deg)
- GNSS file (GNSS_RTK.pos, optional): columns =
  [SOW, Lat(deg), Lon(deg), H(m), sigmaLat(m), sigmaLon(m), sigmaH(m)]  (position-only)

Figures:
1) Position & velocity (navout)
2) Attitude (navout)
3) Position errors w.r.t truth (m)
4) Velocity errors w.r.t truth (m/s)
5) Attitude errors w.r.t truth (deg, unwrapped to [-180, 180])
6) Horizontal trajectory (truth vs navout + optional GNSS scatter)

Author: peanut-nav (adapted)
Date: 2025-09-07
"""

import os
import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt

# --- Plot style (kept similar to your old script) ---
plt.style.use('seaborn-whitegrid')
plt.rcParams['figure.dpi'] = 150
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['xtick.labelsize'] = 8
plt.rcParams['ytick.labelsize'] = 8

Re = 6378135.072
deg2rad = np.pi / 180.0
rad2deg = 180.0 / np.pi

# ---------- Loaders ----------
def load_navoutq(path, imu_rate=200.0, sim_time=600.0):
    """Load navigation results saved by C++ SaveResults."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    n = data.shape[0]
    t = np.linspace(0.0, sim_time, n)
    return {
        "time": t,
        "lat":   data[:, 1],
        "lon":   data[:, 2],
        "h":     data[:, 3],
        "vE":    data[:, 4],
        "vN":    data[:, 5],
        "vU":    data[:, 6],
        "yaw":   data[:, 7],   # already heading convention from C++ (360 - yawDeg)
        "pitch": data[:, 8],
        "roll":  data[:, 9],
    }

def load_truth_nav(path):
    """Load truth.nav (Week SOW Lat Lon H vN vE vD Roll Pitch Yaw)."""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    week   = data[:, 0]
    sow    = data[:, 1]
    lat    = data[:, 2]
    lon    = data[:, 3]
    h      = data[:, 4]
    vN     = data[:, 5]
    vE     = data[:, 6]
    vD     = data[:, 7]
    roll   = data[:, 8]
    pitch  = data[:, 9]
    yaw    = data[:,10]

    heading = (360.0 - np.mod(yaw, 360.0))  # match C++ convention

    return {
        "week": week,
        "sow": sow,
        "lat": lat,
        "lon": lon,
        "h":   h,
        "vE":  vE,
        "vN":  vN,
        "vU":  -vD,   # N,E,D -> E,N,U
        "roll":  roll,
        "pitch": pitch,
        "yaw":   heading,  # heading convention
    }

def load_gnss_pos(path):
    """Load GNSS_RTK.pos (SOW Lat Lon H sigmaLat sigmaLon sigmaH)."""
    if not path or not os.path.exists(path):
        return None
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data[None, :]
    return {
        "sow": data[:, 0],
        "lat": data[:, 1],
        "lon": data[:, 2],
        "h":   data[:, 3],
        "sLat": data[:, 4],
        "sLon": data[:, 5],
        "sH":   data[:, 6],
    }

# ---------- Helpers ----------
def angle_diff_deg(a, b):
    """Minimal difference a - b in degrees, wrapped to [-180, 180]."""
    d = (a - b + 180.0) % 360.0 - 180.0
    # For values exactly at 180, choose -180 for stability
    d = np.where(d == 180.0, -180.0, d)
    return d

def calc_pos_errors_m(lat_deg, lon_deg, lat_ref_deg, lon_ref_deg):
    """Convert (lat, lon) errors to meters, with lon scaled by cos(lat_ref)."""
    dlat_m = (lat_deg - lat_ref_deg) * deg2rad * Re
    # use instantaneous reference latitude for better scaling
    clat = np.cos(lat_ref_deg * deg2rad)
    dlon_m = (lon_deg - lon_ref_deg) * deg2rad * Re * clat
    return dlat_m, dlon_m

def clamp_window(up, down, maxN):
    up = max(0, min(int(up), maxN - 1))
    down = max(up + 1, min(int(down), maxN))
    return up, down

def unify_lengths(state, truth):
    """Truncate both to common length (min of both), return (state, truth, N)."""
    Ns = len(state["lat"])
    Nt = len(truth["lat"])
    N  = min(Ns, Nt)
    for k in state:
        state[k] = state[k][:N]
    for k in truth:
        truth[k] = truth[k][:N]
    return state, truth, N

# ---------- Visualization ----------
def visualize(state, truth, gnss=None, output_dir=None, title_prefix=""):
    t = state["time"]
    # 1) Position & Velocity
    fig1 = plt.figure(figsize=(12, 8))
    fig1.suptitle(f'{title_prefix}Position and Velocity (navout)', fontsize=14)

    ax = fig1.add_subplot(2, 3, 1); ax.plot(t, state["lat"]);  ax.set_title("Latitude");  ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax = fig1.add_subplot(2, 3, 2); ax.plot(t, state["lon"]);  ax.set_title("Longitude"); ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax = fig1.add_subplot(2, 3, 3); ax.plot(t, state["h"]);    ax.set_title("Height");    ax.set_xlabel("Time (s)"); ax.set_ylabel("m")
    ax = fig1.add_subplot(2, 3, 4); ax.plot(t, state["vE"]);   ax.set_title("vE");        ax.set_xlabel("Time (s)"); ax.set_ylabel("m/s")
    ax = fig1.add_subplot(2, 3, 5); ax.plot(t, state["vN"]);   ax.set_title("vN");        ax.set_xlabel("Time (s)"); ax.set_ylabel("m/s")
    ax = fig1.add_subplot(2, 3, 6); ax.plot(t, state["vU"]);   ax.set_title("vU");        ax.set_xlabel("Time (s)"); ax.set_ylabel("m/s")
    fig1.tight_layout(rect=[0, 0, 1, 0.96])

    # 2) Attitude
    fig2 = plt.figure(figsize=(8, 10))
    fig2.suptitle(f'{title_prefix}Attitude (navout)', fontsize=14)
    ax = fig2.add_subplot(3, 1, 1); ax.plot(t, state["pitch"]); ax.set_title("Pitch"); ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax = fig2.add_subplot(3, 1, 2); ax.plot(t, state["roll"]);  ax.set_title("Roll");  ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax = fig2.add_subplot(3, 1, 3); ax.plot(t, state["yaw"]);   ax.set_title("Yaw");   ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    fig2.tight_layout(rect=[0, 0, 1, 0.96])

    # 3) Position Errors vs truth (m)
    dlat_m, dlon_m = calc_pos_errors_m(state["lat"], state["lon"], truth["lat"], truth["lon"])
    dh_m = state["h"] - truth["h"]
    fig3 = plt.figure(figsize=(8, 10))
    fig3.suptitle(f'{title_prefix}Position Errors (vs truth)', fontsize=14)
    ax = fig3.add_subplot(3, 1, 1); ax.plot(t, dlat_m); ax.set_title("Latitude Error");  ax.set_xlabel("Time (s)"); ax.set_ylabel("m")
    ax = fig3.add_subplot(3, 1, 2); ax.plot(t, dlon_m); ax.set_title("Longitude Error"); ax.set_xlabel("Time (s)"); ax.set_ylabel("m")
    ax = fig3.add_subplot(3, 1, 3); ax.plot(t, dh_m);   ax.set_title("Height Error");    ax.set_xlabel("Time (s)"); ax.set_ylabel("m")
    fig3.tight_layout(rect=[0, 0, 1, 0.96])

    # 4) Velocity Errors vs truth (m/s)
    dvE = state["vE"] - truth["vE"]
    dvN = state["vN"] - truth["vN"]
    dvU = state["vU"] - truth["vU"]
    fig4 = plt.figure(figsize=(8, 10))
    fig4.suptitle(f'{title_prefix}Velocity Errors (vs truth)', fontsize=14)
    ax = fig4.add_subplot(3, 1, 1); ax.plot(t, dvE); ax.set_title("vE Error"); ax.set_xlabel("Time (s)"); ax.set_ylabel("m/s")
    ax = fig4.add_subplot(3, 1, 2); ax.plot(t, dvN); ax.set_title("vN Error"); ax.set_xlabel("Time (s)"); ax.set_ylabel("m/s")
    ax = fig4.add_subplot(3, 1, 3); ax.plot(t, dvU); ax.set_title("vU Error"); ax.set_xlabel("Time (s)"); ax.set_ylabel("m/s")
    fig4.tight_layout(rect=[0, 0, 1, 0.96])

    # 5) Attitude Errors vs truth (deg) with unwrap to [-180,180]
    dyaw   = angle_diff_deg(state["yaw"],  truth["yaw"])
    dpitch = angle_diff_deg(state["pitch"], truth["pitch"])
    droll  = angle_diff_deg(state["roll"],  truth["roll"])
    fig5 = plt.figure(figsize=(8, 10))
    fig5.suptitle(f'{title_prefix}Attitude Errors (vs truth)', fontsize=14)
    ax = fig5.add_subplot(3, 1, 1); ax.plot(t, dpitch); ax.set_title("Pitch Error"); ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax = fig5.add_subplot(3, 1, 2); ax.plot(t, droll);  ax.set_title("Roll Error");  ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    ax = fig5.add_subplot(3, 1, 3); ax.plot(t, dyaw);   ax.set_title("Yaw Error");   ax.set_xlabel("Time (s)"); ax.set_ylabel("deg")
    fig5.tight_layout(rect=[0, 0, 1, 0.96])

    # 6) Horizontal trajectory (truth vs navout + GNSS)
    fig6 = plt.figure(figsize=(10, 8))
    plt.plot(truth["lon"], truth["lat"], label="Truth")
    plt.plot(state["lon"], state["lat"],  label="NavOut")
    if gnss is not None:
        plt.scatter(gnss["lon"], gnss["lat"], s=3, alpha=0.5, label="GNSS (pos-only)")
    plt.title(f'{title_prefix}Horizontal Trajectory')
    plt.xlabel("Longitude (deg)")
    plt.ylabel("Latitude (deg)")
    plt.legend()
    plt.grid(True)
    fig6.tight_layout()

    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
        fig1.savefig(os.path.join(output_dir, "fig1_pos_vel.png"))
        fig2.savefig(os.path.join(output_dir, "fig2_attitude.png"))
        fig3.savefig(os.path.join(output_dir, "fig3_pos_errors.png"))
        fig4.savefig(os.path.join(output_dir, "fig4_vel_errors.png"))
        fig5.savefig(os.path.join(output_dir, "fig5_attitude_errors.png"))
        fig6.savefig(os.path.join(output_dir, "fig6_trajectory.png"))

# ---------- Stats ----------
def compute_stats_over_window(state, truth, up=None, down=None):
    N = len(state["lat"])
    if up is None:   up = 0
    if down is None: down = N
    up, down = clamp_window(up, down, N)

    dlat_m, dlon_m = calc_pos_errors_m(state["lat"][up:down], state["lon"][up:down],
                                       truth["lat"][up:down], truth["lon"][up:down])
    dh_m = state["h"][up:down] - truth["h"][up:down]
    dvE  = state["vE"][up:down] - truth["vE"][up:down]
    dvN  = state["vN"][up:down] - truth["vN"][up:down]
    dvU  = state["vU"][up:down] - truth["vU"][up:down]
    dyaw   = angle_diff_deg(state["yaw"][up:down],   truth["yaw"][up:down])
    dpitch = angle_diff_deg(state["pitch"][up:down], truth["pitch"][up:down])
    droll  = angle_diff_deg(state["roll"][up:down],  truth["roll"][up:down])

    def rms(x): return float(np.sqrt(np.mean(np.square(x)))) if len(x) else float("nan")
    stats = {
        "RMS_lat_m":   rms(dlat_m),
        "RMS_lon_m":   rms(dlon_m),
        "RMS_h_m":     rms(dh_m),
        "RMS_vE_mps":  rms(dvE),
        "RMS_vN_mps":  rms(dvN),
        "RMS_vU_mps":  rms(dvU),
        "RMS_yaw_deg":   rms(dyaw),
        "RMS_pitch_deg": rms(dpitch),
        "RMS_roll_deg":  rms(droll),
        "window": (up, down),
        "N": N
    }
    return stats

def print_and_maybe_save_stats(stats, output_dir=None):
    up, down = stats["window"]
    print("\n===== Performance Statistics =====")
    print(f"Window: [{up}, {down}) of N={stats['N']}")
    print(f"Latitude RMS Error:  {stats['RMS_lat_m']:.6f} m")
    print(f"Longitude RMS Error: {stats['RMS_lon_m']:.6f} m")
    print(f"Height RMS Error:    {stats['RMS_h_m']:.6f} m")
    print(f"Yaw RMS Error:       {stats['RMS_yaw_deg']:.6f} °")
    print(f"Pitch RMS Error:     {stats['RMS_pitch_deg']:.6f} °")
    print(f"Roll RMS Error:      {stats['RMS_roll_deg']:.6f} °")

    if output_dir:
        path = os.path.join(output_dir, "performance_statistics.txt")
        with open(path, "w") as f:
            f.write("===== Performance Statistics =====\n")
            f.write(f"Window: [{up}, {down}) of N={stats['N']}\n")
            f.write(f"Latitude RMS Error:  {stats['RMS_lat_m']:.6f} m\n")
            f.write(f"Longitude RMS Error: {stats['RMS_lon_m']:.6f} m\n")
            f.write(f"Height RMS Error:    {stats['RMS_h_m']:.6f} m\n")
            f.write(f"Yaw RMS Error:       {stats['RMS_yaw_deg']:.6f} °\n")
            f.write(f"Pitch RMS Error:     {stats['RMS_pitch_deg']:.6f} °\n")
            f.write(f"Roll RMS Error:      {stats['RMS_roll_deg']:.6f} °\n")
        print(f"Performance statistics saved to: {path}")

# ---------- CLI ----------
def main():
    ap = argparse.ArgumentParser(
        description="Visualizer for new dataset (truth.nav / GNSS_RTK.pos)",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    ap.add_argument("--navoutq", type=str, default="../output/smoothed_EKF_navoutQ.dat",
                    help="navigation results file")
    ap.add_argument("--truth",   type=str, default="../data/truth.nav",
                    help="truth.nav path")
    ap.add_argument("--gnss",    type=str, default="../data/GNSS_RTK.pos",
                    help="GNSS_RTK.pos path (optional)")
    ap.add_argument("--output",  type=str, default="../output",
                    help="directory to save figures/statistics (or omit to just show)")
    ap.add_argument("--show",    action="store_true",
                    help="show figures interactively (in addition to saving if --output)")
    ap.add_argument("--imu-rate", type=float, default=200.0,
                    help="IMU sampling rate [Hz] (for nav time axis)")
    ap.add_argument("--sim-time", type=float, default=600.0,
                    help="Total simulation time [s] (for nav time axis)")
    ap.add_argument("--eval-up",   type=int, default=None,
                    help="evaluation window start index (default: 0)")
    ap.add_argument("--eval-down", type=int, default=None,
                    help="evaluation window end index (default: N)")
    args = ap.parse_args()

    if args.output:
        os.makedirs(args.output, exist_ok=True)

    # Load
    state = load_navoutq(args.navoutq, imu_rate=args.imu_rate, sim_time=args.sim_time)
    truth = load_truth_nav(args.truth)
    gnss  = load_gnss_pos(args.gnss) if args.gnss and os.path.exists(args.gnss) else None

    # Align lengths safely
    state, truth, N = unify_lengths(state, truth)
    if gnss is not None:
        # no hard alignment for GNSS; it is displayed as scatter only

        pass

    # Visualize & Stats
    visualize(state, truth, gnss=gnss, output_dir=args.output, title_prefix="")
    stats = compute_stats_over_window(state, truth, up=args.eval_up, down=args.eval_down)
    print_and_maybe_save_stats(stats, output_dir=args.output)

    if args.show or not args.output:
        plt.show()
    else:
        print(f"All figures saved to: {args.output}")

if __name__ == "__main__":
    main()

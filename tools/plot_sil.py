#!/usr/bin/env python3
"""
plot_sil.py - Visualization and Analysis Tool for QFC-1 SIL Flight Simulation Telemetry
Author: Rohith B Narasimhamurthy
"""

import sys
import csv
from pathlib import Path

def print_ascii_metrics(csv_path: Path):
    if not csv_path.exists():
        print(f"Error: Telemetry file {csv_path} does not exist.")
        return

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    scenarios = set(r["scenario"] for r in rows)
    print("\n" + "=" * 65)
    print("      QFC-1 SIL FLIGHT DYNAMICS - TELEMETRY AUDIT REPORT")
    print("=" * 65)

    for sc in ["TiltRecovery", "GustRejection", "SetpointTracking", "EmergencyTripwire"]:
        sc_rows = [r for r in rows if r["scenario"] == sc]
        if not sc_rows:
            continue

        times = [float(r["time_s"]) for r in sc_rows]
        true_rolls = [float(r["true_roll_deg"]) for r in sc_rows]
        true_pitches = [float(r["true_pitch_deg"]) for r in sc_rows]
        est_rolls = [float(r["est_roll_deg"]) for r in sc_rows]
        est_pitches = [float(r["est_pitch_deg"]) for r in sc_rows]
        states = [int(r["state"]) for r in sc_rows]
        faults = [int(r["fault"]) for r in sc_rows]

        print(f"\n[+] Scenario: {sc}")
        print(f"    - Duration           : {times[-1] - times[0]:.3f} s ({len(sc_rows)} ms steps)")
        print(f"    - Peak Roll Range    : [{min(true_rolls):+6.2f} deg, {max(true_rolls):+6.2f} deg]")
        print(f"    - Peak Pitch Range   : [{min(true_pitches):+6.2f} deg, {max(true_pitches):+6.2f} deg]")
        print(f"    - Final True Attitude: Roll={true_rolls[-1]:+6.2f} deg, Pitch={true_pitches[-1]:+6.2f} deg")
        print(f"    - Final Estimator    : Roll={est_rolls[-1]:+6.2f} deg, Pitch={est_pitches[-1]:+6.2f} deg")
        print(f"    - Final Safety State : State={states[-1]} (0=DISARM, 1=ARM, 2=FAULT), Fault={faults[-1]}")

        # ASCII mini sparkline for roll response
        if len(true_rolls) >= 20:
            sample_step = len(true_rolls) // 20
            sampled = true_rolls[::sample_step][:20]
            spark = "".join("^" if v > 10 else "+" if v > 2 else "v" if v < -10 else "-" if v < -2 else "." for v in sampled)
            print(f"    - Attitude Profile   : [{spark}]")

    print("\n" + "=" * 65 + "\n")

def try_generate_png(csv_path: Path, output_png: Path):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[INFO] matplotlib is not installed. Terminal ASCII audit complete.")
        return

    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    fig, axs = plt.subplots(4, 1, figsize=(12, 14), sharex=False)
    fig.suptitle("QFC-1 Flight Controller: 6-DOF Software-in-the-Loop Flight Verification", fontsize=14, fontweight="bold")

    colors = {"TiltRecovery": "#1f77b4", "GustRejection": "#ff7f0e", "SetpointTracking": "#2ca02c", "EmergencyTripwire": "#d62728"}

    for sc in ["TiltRecovery", "GustRejection", "SetpointTracking", "EmergencyTripwire"]:
        sc_rows = [r for r in rows if r["scenario"] == sc]
        if not sc_rows:
            continue

        t = [float(r["time_s"]) for r in sc_rows]
        true_r = [float(r["true_roll_deg"]) for r in sc_rows]
        tgt_r = [float(r["target_roll_deg"]) for r in sc_rows]
        m1 = [float(r["m1"]) for r in sc_rows]
        m2 = [float(r["m2"]) for r in sc_rows]
        m3 = [float(r["m3"]) for r in sc_rows]
        m4 = [float(r["m4"]) for r in sc_rows]

        # Plot 1: Tilt Recovery
        if sc == "TiltRecovery":
            axs[0].plot(t, true_r, label="True Roll (deg)", color=colors[sc], lw=1.8)
            axs[0].plot(t, [float(r["true_pitch_deg"]) for r in sc_rows], label="True Pitch (deg)", color="#17becf", lw=1.5, ls="--")
            axs[0].axhline(0, color="gray", ls=":", alpha=0.7)
            axs[0].set_title("Scenario 1: Initial Severe Tilt Recovery (+20 deg Roll, -15 deg Pitch)")
            axs[0].set_ylabel("Attitude (deg)")
            axs[0].legend(loc="upper right")
            axs[0].grid(True, alpha=0.3)

        # Plot 2: Gust Rejection
        if sc == "GustRejection":
            axs[1].plot(t, true_r, label="True Roll (deg)", color=colors[sc], lw=1.8)
            axs[1].axvspan(0.500, 0.580, color="red", alpha=0.15, label="Wind Gust Impulse (0.04 N*m)")
            axs[1].axhline(0, color="gray", ls=":", alpha=0.7)
            axs[1].set_title("Scenario 2: Wind Gust Torque Impulse Rejection (500 ms - 580 ms)")
            axs[1].set_ylabel("Attitude (deg)")
            axs[1].legend(loc="upper right")
            axs[1].grid(True, alpha=0.3)

        # Plot 3: Setpoint Tracking
        if sc == "SetpointTracking":
            axs[2].plot(t, true_r, label="True Roll (deg)", color=colors[sc], lw=1.8)
            axs[2].plot(t, tgt_r, label="Target Setpoint (deg)", color="black", lw=1.5, ls="--")
            axs[2].set_title("Scenario 3: Dynamic Attitude Setpoint Tracking (0 -> +12 -> -12 -> 0 deg)")
            axs[2].set_ylabel("Attitude (deg)")
            axs[2].legend(loc="upper right")
            axs[2].grid(True, alpha=0.3)

        # Plot 4: Emergency Tripwire
        if sc == "EmergencyTripwire":
            axs[3].plot(t, true_r, label="True Roll (deg)", color=colors[sc], lw=1.8)
            axs[3].plot(t, m1, label="Motor 1 (PWM)", color="#9467bd", lw=1.2)
            axs[3].plot(t, m2, label="Motor 2 (PWM)", color="#8c564b", lw=1.2)
            axs[3].axhline(90, color="darkred", ls="--", label="Divergence Tripwire (90 deg)")
            axs[3].set_title("Scenario 4: Extreme Inversion Emergency Tripwire & Motor Shutdown")
            axs[3].set_xlabel("Simulation Time (s)")
            axs[3].set_ylabel("Roll (deg) / PWM")
            axs[3].legend(loc="upper left")
            axs[3].grid(True, alpha=0.3)

    plt.tight_layout()
    output_png.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(output_png, dpi=200)
    print(f"[+] High-resolution flight visualization saved to: {output_png}")

def main():
    csv_path = Path("./simulation/logs/sil_telemetry.csv")
    output_png = Path("./simulation/logs/sil_flight_telemetry.png")

    print_ascii_metrics(csv_path)
    try_generate_png(csv_path, output_png)

if __name__ == "__main__":
    main()

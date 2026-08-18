import csv
import math
import os

import matplotlib.pyplot as plt
import numpy as np


INPUT_FILE = "Final_Data.csv"


def parse_performance_table(csv_path):
    rows = []

    with open(csv_path, newline="", encoding="utf-8", errors="replace") as f:
        reader = csv.reader(f)
        all_rows = list(reader)

    start_idx = None
    for i, row in enumerate(all_rows):
        if row and len(row) >= 8 and row[0].strip() == "Thrust (N)":
            start_idx = i + 1
            break

    if start_idx is None:
        raise ValueError("Could not find the performance table header starting with 'Thrust (N)'.")

    for row in all_rows[start_idx:]:
        if not row or not row[0].strip():
            break

        try:
            thrust_n = float(row[0])
            ideal_power = float(row[1])
            actual_elec_power = float(row[2])
            actual_mech_power = float(row[3])
            rad_per_sec = float(row[4])
            figure_of_merit = float(row[5])
            ct = float(row[6])
            system_eff = float(row[7])
        except (ValueError, IndexError):
            # Stop when data rows end or become non-numeric.
            break

        rpm = rad_per_sec * 60.0 / (2.0 * math.pi)

        rows.append(
            {
                "Thrust (N)": thrust_n,
                "Ideal Power": ideal_power,
                "Actual Electrical Power": actual_elec_power,
                "Actual Mech. Power": actual_mech_power,
                "Radians/second": rad_per_sec,
                "RPM": rpm,
                "Figure Of Merit": figure_of_merit,
                "CT": ct,
                "System Efficiency": system_eff,
            }
        )

    if not rows:
        raise ValueError("Performance table was found, but no numeric rows were parsed.")

    return rows


def plot_xy(x, y, xlabel, ylabel, title, out_file, fit_degree=None):
    plt.figure(figsize=(8, 5))
    plt.plot(x, y, "o-", linewidth=1.5, markersize=5, label="Data")

    if fit_degree is not None and len(x) > fit_degree:
        coeff = np.polyfit(x, y, fit_degree)
        xfit = np.linspace(float(np.min(x)), float(np.max(x)), 300)
        yfit = np.polyval(coeff, xfit)
        plt.plot(xfit, yfit, "--", linewidth=1.5, label=f"Poly fit (deg {fit_degree})")

    plt.xlabel(xlabel, fontweight='bold', fontsize=11)
    plt.ylabel(ylabel, fontweight='bold', fontsize=11)
    plt.title(title, fontweight='bold', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_file, dpi=150)
    plt.close()


def main():
    script_dir = os.path.dirname(__file__)
    data_dir = os.path.join(script_dir, '..', 'data')
    results_dir = os.path.join(script_dir, '..', 'results')

    data = parse_performance_table(os.path.join(data_dir, INPUT_FILE))

    rpm = np.array([d["RPM"] for d in data], dtype=float)
    thrust_n = np.array([d["Thrust (N)"] for d in data], dtype=float)
    mech_power = np.array([d["Actual Mech. Power"] for d in data], dtype=float)
    fom = np.array([d["Figure Of Merit"] for d in data], dtype=float)
    system_eff = np.array([d["System Efficiency"] for d in data], dtype=float)
    ct = np.array([d["CT"] for d in data], dtype=float)

    # 1) Core Performance Plots
    plot_xy(
        rpm,
        thrust_n,
        "RPM (rev/min)",
        "Thrust (N)",
        "Thrust (N) vs RPM",
        os.path.join(results_dir, "thrust_vs_rpm.png"),
        fit_degree=2,
    )

    plot_xy(
        rpm,
        mech_power,
        "RPM (rev/min)",
        "Actual Mech. Power (W)",
        "Actual Mech. Power (W) vs RPM",
        os.path.join(results_dir, "mech_power_vs_rpm.png"),
        fit_degree=3,
    )

    # 2) Efficiency Plots
    plot_xy(
        thrust_n,
        fom,
        "Thrust (N)",
        "Figure Of Merit (dimensionless)",
        "Figure Of Merit vs Thrust (N)",
        os.path.join(results_dir, "fom_vs_thrust.png"),
        fit_degree=None,
    )

    plot_xy(
        rpm,
        system_eff,
        "RPM (rev/min)",
        "System Efficiency (dimensionless)",
        "System Efficiency vs RPM",
        os.path.join(results_dir, "system_efficiency_vs_rpm.png"),
        fit_degree=None,
    )

    # 3) Non-Dimensional Plot
    plot_xy(
        rpm,
        ct,
        "RPM (rev/min)",
        "CT (dimensionless)",
        "CT vs RPM",
        os.path.join(results_dir, "ct_vs_rpm.png"),
        fit_degree=None,
    )

    print("Saved plots:")
    print("  thrust_vs_rpm.png")
    print("  mech_power_vs_rpm.png")
    print("  fom_vs_thrust.png")
    print("  system_efficiency_vs_rpm.png")
    print("  ct_vs_rpm.png")


if __name__ == "__main__":
    main()

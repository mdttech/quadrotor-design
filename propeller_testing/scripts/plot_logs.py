import os
import glob
import csv
import math
import numpy as np
import matplotlib.pyplot as plt


US_POINTS = np.array([1000, 1100, 1150, 1200, 1250, 1300, 1350, 1400, 1450, 1475], dtype=float)
RPM_POINTS = np.array([0, 2130, 5740, 8000, 9600, 10750, 11850, 12770, 14100, 14600], dtype=float)


def read_log(path):
    times = []
    esc = []
    with open(path, newline='') as f:
        reader = csv.reader(f)
        header = next(reader)
        # find indices
        try:
            t_idx = header.index('Time (s)')
        except ValueError:
            t_idx = 0
        try:
            esc_idx = header.index('ESC signal (µs)')
        except ValueError:
            # fallback to second column
            esc_idx = 1

        for row in reader:
            if not row:
                continue
            try:
                t = float(row[t_idx])
            except Exception:
                continue
            # ESC may be empty
            try:
                e = float(row[esc_idx])
            except Exception:
                e = math.nan
            times.append(t)
            esc.append(e)

    return np.array(times), np.array(esc)


def esc_to_rpm(esc_us):
    # interpolate, allow nan to propagate
    mask = ~np.isnan(esc_us)
    rpm = np.full_like(esc_us, np.nan, dtype=float)
    if np.any(mask):
        rpm[mask] = np.interp(esc_us[mask], US_POINTS, RPM_POINTS)
    return rpm


def plot_file(path, results_dir):
    times, esc = read_log(path)
    if len(times) == 0:
        print('No data in', path)
        return
    rpm = esc_to_rpm(esc)

    base = os.path.splitext(os.path.basename(path))[0]

    # Time vs RPM
    plt.figure(figsize=(10,4))
    plt.plot(times, rpm, '-', lw=1)
    plt.xlabel('Time (s)')
    plt.ylabel('RPM (mapped from ESC µs)')
    plt.title(base + ' — Time vs RPM')
    plt.grid(True)
    out1 = os.path.join(results_dir, base + '_time_rpm.png')
    plt.tight_layout()
    plt.savefig(out1)
    plt.close()

    # ESC µs vs RPM
    plt.figure(figsize=(6,4))
    # scatter measured points
    plt.scatter(esc, rpm, s=6, alpha=0.7)
    # plot calibration curve
    xs = np.linspace(np.nanmin(US_POINTS), np.nanmax(US_POINTS), 300)
    ys = np.interp(xs, US_POINTS, RPM_POINTS)
    plt.plot(xs, ys, 'r--', lw=1)
    plt.xlabel('ESC signal (µs)')
    plt.ylabel('RPM')
    plt.title(base + ' — ESC µs vs RPM')
    plt.grid(True)
    out2 = os.path.join(results_dir, base + '_esc_rpm.png')
    plt.tight_layout()
    plt.savefig(out2)
    plt.close()

    print('Saved', out1, 'and', out2)


def plot_combined(paths, results_dir):
    all_times = []
    all_rpm = []
    for p in paths:
        t, e = read_log(p)
        if len(t)==0:
            continue
        r = esc_to_rpm(e)
        all_times.append(t)
        all_rpm.append(r)

    if not all_times:
        print('No data to plot combined')
        return

    plt.figure(figsize=(10,4))
    for t, r in zip(all_times, all_rpm):
        plt.plot(t, r, lw=1)
    plt.xlabel('Time (s)')
    plt.ylabel('RPM')
    plt.title('Combined — Time vs RPM')
    plt.grid(True)
    plt.tight_layout()
    out = os.path.join(results_dir, 'combined_time_rpm.png')
    plt.savefig(out)
    plt.close()
    print('Saved', out)


def main():
    script_dir = os.path.dirname(__file__)
    data_dir = os.path.join(script_dir, '..', 'data')
    results_dir = os.path.join(script_dir, '..', 'results')
    paths = sorted(glob.glob(os.path.join(data_dir, 'Log_*.csv')))
    if not paths:
        print('No Log_*.csv files found in', data_dir)
        return
    for p in paths:
        print('Processing', p)
        plot_file(p, results_dir)
    if len(paths) > 1:
        plot_combined(paths, results_dir)


if __name__ == '__main__':
    main()

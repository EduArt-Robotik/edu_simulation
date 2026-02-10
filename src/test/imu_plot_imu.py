#!/usr/bin/env python3
"""Plot IMU data from a CSV file created by imu_validation_test.py.

Usage:
  python3 imu_plot_imu.py --csv /path/to/imu_validation_*.csv
"""

import argparse
import csv

import matplotlib.pyplot as plt


def load_csv(csv_path: str):
    t = []
    wx = []
    wy = []
    wz = []
    ax = []
    ay = []
    az = []

    with open(csv_path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                t.append(float(row["time"]))
                wx.append(float(row["angular_vel_x"]))
                wy.append(float(row["angular_vel_y"]))
                wz.append(float(row["angular_vel_z"]))
                ax.append(float(row["linear_accel_x"]))
                ay.append(float(row["linear_accel_y"]))
                az.append(float(row["linear_accel_z"]))
            except (KeyError, ValueError):
                continue

    return t, wx, wy, wz, ax, ay, az

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot IMU data from a CSV file created by imu_validation_test.py."
    )
    parser.add_argument("--csv", required=True, help="Path to IMU CSV file")
    args = parser.parse_args()

    t, wx, wy, wz, ax, ay, az = load_csv(args.csv)
    if not t:
        raise SystemExit("No IMU rows found in CSV. Check the input file.")

    fig, (ax_w, ax_a) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax_w.set_title("IMU Angular Velocity")
    ax_w.set_ylabel("rad/s")
    ax_a.set_title("IMU Linear Acceleration")
    ax_a.set_ylabel("m/s^2")
    ax_a.set_xlabel("time [s]")

    ax_w.plot(t, wx, label="wx")
    ax_w.plot(t, wy, label="wy")
    ax_w.plot(t, wz, label="wz")
    ax_w.legend(loc="upper right")

    ax_a.plot(t, ax, label="ax")
    ax_a.plot(t, ay, label="ay")
    ax_a.plot(t, az, label="az")
    ax_a.legend(loc="upper right")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()

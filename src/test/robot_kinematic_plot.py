#!/usr/bin/env python3
"""Plot robot pose data from a CSV file created by its internal logger.

Usage:
  python3 robot_kinematic_plot.py --csv /path/to/robot_pose_log.csv
"""

import argparse
import csv
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

def load_csv(csv_path: str):
    t = []
    x = []
    y = []
    z = []
    qx = []
    qy = []
    qz = []
    qw = []

    with open(csv_path, "r", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                t.append(float(row["time"]))
                x.append(float(row["position_x"]))
                y.append(float(row["position_y"]))
                z.append(float(row["position_z"]))
                qx.append(float(row["orientation_x"]))
                qy.append(float(row["orientation_y"]))
                qz.append(float(row["orientation_z"]))
                qw.append(float(row["orientation_w"]))
            except (KeyError, ValueError):
                continue

    return t, x, y, z, qx, qy, qz, qw

def calculate_linear_velocity(t, x, y, z):
    vx = [0.0] # Add first element for linear velocity to make it the same length as t
    vy = [0.0] # Add first element for linear velocity to make it the same length as t
    vz = [0.0] # Add first element for linear velocity to make it the same length as t

    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        if dt > 0:
            vx.append((x[i] - x[i-1]) / dt)
            vy.append((y[i] - y[i-1]) / dt)
            vz.append((z[i] - z[i-1]) / dt)
        else:
            vx.append(0.0)
            vy.append(0.0)
            vz.append(0.0)

    return vx, vy, vz

def calculate_linear_acceleration(t, vx, vy, vz):
    ax = [0.0] # Add first element for linear acceleration to make it the same length as t
    ay = [0.0] # Add first element for linear acceleration to make it the same length as t
    az = [0.0] # Add first element for linear acceleration to make it the same length as t

    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        if dt > 0:
            ax.append((vx[i] - vx[i-1]) / dt)
            ay.append((vy[i] - vy[i-1]) / dt)
            az.append((vz[i] - vz[i-1]) / dt)
        else:
            ax.append(0.0)
            ay.append(0.0)
            az.append(0.0)

    return ax, ay, az

def angular_velocity_from_quaternions(q1, q2, dt):
    """
    Calculate angular velocity from two quaternions.
    
    Args:
        q1: Quaternion at time t [x, y, z, w] (or [w, x, y, z])
        q2: Quaternion at time t+dt [x, y, z, w]
        dt: Time difference in seconds
    
    Returns:
        Angular velocity as [wx, wy, wz] in rad/s
    """
    r1 = Rotation.from_quat(q1)
    r2 = Rotation.from_quat(q2)
    
    # Relative rotation: dR = R2 * R1^-1
    r_rel = r2 * r1.inv()
    
    # Convert to axis-angle (compact form: axis * angle)
    rotvec = r_rel.as_rotvec()
    
    # Angular velocity = rotation vector / time
    omega = rotvec / dt
    return omega

def calculate_angular_velocity(t, qx, qy, qz, qw):
    wx = [0.0] # Add first element for angular velocity to make it the same length as t
    wy = [0.0] # Add first element for angular velocity to make it the same length as t
    wz = [0.0] # Add first element for angular velocity to make it the same length as t

    for i in range(1, len(t)):
        dt = t[i] - t[i-1]
        if dt > 0:
            c_wx, c_wy, c_wz = angular_velocity_from_quaternions(
                [qx[i-1], qy[i-1], qz[i-1], qw[i-1]],
                [qx[i], qy[i], qz[i], qw[i]],
                dt
            )

            wx.append(c_wx)
            wy.append(c_wy)
            wz.append(c_wz)
        else:
            wx.append(0.0)
            wy.append(0.0)
            wz.append(0.0)

    return wx, wy, wz

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot robot pose data from a CSV file created by its internal logger."
    )
    parser.add_argument("--csv", required=True, help="Path to robot pose CSV file")
    args = parser.parse_args()

    t, x, y, z, qx, qy, qz, qw = load_csv(args.csv)

    if not t:
        raise SystemExit("No robot pose rows found in CSV. Check the input file.")

    vx, vy, vz = calculate_linear_velocity(t, x, y, z)
    ax, ay, az = calculate_linear_acceleration(t, vx, vy, vz)
    wx, wy, wz = calculate_angular_velocity(t, qx, qy, qz, qw)

    fig, (ax_pos, ax_vel, ax_acc, ax_ang_vel) = plt.subplots(4, 1, figsize=(10, 6), sharex=True)
    ax_pos.set_title("Robot Position")
    ax_pos.set_ylabel("Position [m]")
    ax_vel.set_title("Robot Velocity")
    ax_vel.set_ylabel("Velocity [m/s]")
    ax_acc.set_title("Robot Linear Acceleration")
    ax_acc.set_ylabel("Linear Acceleration [m/s^2]")

    ax_ang_vel.set_title("Robot Angular Velocity")
    ax_ang_vel.set_ylabel("Angular Velocity [rad/s]")
    ax_ang_vel.set_xlabel("time [s]")

    ax_pos.plot(t, x, label="x")
    ax_pos.plot(t, y, label="y")
    ax_pos.plot(t, z, label="z")
    ax_pos.legend(loc="upper right")

    ax_vel.plot(t, vx, label="vx")
    ax_vel.plot(t, vy, label="vy")
    ax_vel.plot(t, vz, label="vz")

    ax_acc.plot(t, ax, label="ax")
    ax_acc.plot(t, ay, label="ay")
    ax_acc.plot(t, az, label="az")
    ax_acc.legend(loc="upper right")

    ax_ang_vel.plot(t, wx, label="wx")
    ax_ang_vel.plot(t, wy, label="wy")
    ax_ang_vel.plot(t, wz, label="wz")
    ax_ang_vel.legend(loc="upper right")

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()

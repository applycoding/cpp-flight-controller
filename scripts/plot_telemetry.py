#!/usr/bin/env python3
"""Plot a telemetry CSV written by fc_sim_demo."""

import argparse
import csv
from pathlib import Path


def load(path: Path) -> dict[str, list[float]]:
    columns: dict[str, list[float]] = {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise SystemExit(f"{path} has no header")
        for name in reader.fieldnames:
            columns[name] = []
        for row in reader:
            for name in reader.fieldnames:
                columns[name].append(float(row[name]))
    if not columns["t_s"]:
        raise SystemExit(f"{path} has no samples")
    return columns


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="telemetry CSV from fc_sim_demo")
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("docs/closed_loop.png"),
        help="image path to write",
    )
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    data = load(args.csv)
    t = data["t_s"]

    figure, axes = plt.subplots(4, 1, figsize=(9.2, 10.4), sharex=True)
    figure.suptitle("Closed-loop quad simulation", fontsize=13)

    attitude = axes[0]
    series = (
        ("roll", "#1a73e8"),
        ("pitch", "#e37400"),
        ("yaw", "#188038"),
    )
    for name, color in series:
        attitude.plot(t, data[f"{name}_sp_deg"], color=color, linewidth=1.0, alpha=0.35)
        attitude.plot(t, data[f"{name}_deg"], color=color, linewidth=1.45, label=f"{name}")
        attitude.plot(
            t, data[f"{name}_ekf_deg"], color=color, linewidth=1.0, linestyle="--", label=f"{name} EKF"
        )
    attitude.plot(
        t,
        data["roll_gyro_deg"],
        color="#d93025",
        linewidth=1.0,
        linestyle=":",
        label="roll, gyro only",
    )
    attitude.set_ylabel("deg")
    attitude.set_title("Attitude (pale lines are setpoints)")
    attitude.legend(ncol=4, fontsize=8, loc="upper right")

    altitude = axes[1]
    altitude.plot(t, data["alt_sp_m"], color="#9aa0a6", linewidth=1.2, label="setpoint")
    altitude.plot(t, data["alt_m"], color="#1a73e8", linewidth=1.4, label="truth")
    altitude.plot(t, data["alt_est_m"], color="#1a73e8", linewidth=1.0, linestyle="--", label="baro KF")
    altitude.set_ylabel("m")
    altitude.set_title("Altitude (positive up)")
    altitude.legend(ncol=3, fontsize=8, loc="lower right")

    bias = axes[2]
    bias.plot(t, data["bias_x"], color="#1a73e8", linewidth=1.2, linestyle=":", label="true x")
    bias.plot(t, data["bias_ekf_x"], color="#1a73e8", linewidth=1.3, label="EKF x")
    bias.plot(t, data["bias_y"], color="#e37400", linewidth=1.2, linestyle=":", label="true y")
    bias.plot(t, data["bias_ekf_y"], color="#e37400", linewidth=1.3, label="EKF y")
    bias.plot(t, data["bias_z"], color="#188038", linewidth=1.2, linestyle=":", label="true z")
    bias.plot(t, data["bias_ekf_z"], color="#188038", linewidth=1.3, label="EKF z")
    bias.set_ylabel("rad/s")
    bias.set_title("Gyro bias")
    bias.legend(ncol=6, fontsize=8, loc="upper right")

    motors = axes[3]
    for index, color in enumerate(("#1a73e8", "#e37400", "#188038", "#a142f4")):
        motors.plot(t, data[f"m{index}_n"], color=color, linewidth=1.1, label=f"motor {index}")
    motors.set_ylabel("N")
    motors.set_xlabel("time (s)")
    motors.set_title("Quad-X motor thrust")
    motors.legend(ncol=4, fontsize=8, loc="upper right")

    for axis in axes:
        axis.grid(True, alpha=0.35)
        axis.margins(x=0.0)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.tight_layout()
    figure.savefig(args.output, dpi=140)
    print(f"Wrote {args.output}")


if __name__ == "__main__":
    main()

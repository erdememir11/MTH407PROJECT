"""
Detailed figure generator for the 4-sensor A1 before/after analysis.

This script does not change the tracking model. It imports the current
simulation, TDOA, LSE and EKF functions from agv_tdoa_ekf.py and produces
separate analysis folders:

1. outputs/analysis_4_sensor_a1_before
   - A1 is kept at the old positions used before the latest revision.
   - One figure is generated for each 4-sensor geometry.
   - A summary RMSE bar chart and CSV are also generated.

2. outputs/analysis_4_sensor_a1_after
   - A1 is fixed at the left end of B3: (0, 10).
   - One figure is generated for each 4-sensor geometry.
   - A summary RMSE bar chart and CSV are also generated.

3. outputs/analysis_sensor_count_4_to_10
   - The current A1=(0,10) sensor pool is used.
   - One figure is generated for each sensor count from 4 to 10.
   - Summary plots show how RMSE and geometry conditioning change as sensors
     are added.

Run:
    python generate_analysis_figures.py
"""

from __future__ import annotations

from pathlib import Path
import csv

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from agv_tdoa_ekf import (
    Config,
    MISSION_POINTS,
    SENSOR_GEOMETRIES_4,
    SENSOR_POOL_MAX_10,
    DROPOFF_INDEX,
    GATE2_INDEX,
    PARKING_START_INDEX,
    PICKUP_INDEX,
    evaluate_layout,
    inflated_obstacles,
    monte_carlo_layout,
    simulate_robot_motion,
)


A1_BEFORE_GEOMETRIES_4 = {
    "G1_corner_coverage": np.array(
        [[1.0, 1.0], [49.0, 1.0], [49.0, 29.0], [1.0, 29.0]],
        dtype=float,
    ),
    "G2_task_oriented": np.array(
        [[3.0, 27.5], [24.0, 27.5], [28.0, 15.5], [48.0, 11.0]],
        dtype=float,
    ),
    "G3_poor_same_wall": np.array(
        [[2.0, 2.0], [17.0, 2.0], [33.0, 2.0], [48.0, 2.0]],
        dtype=float,
    ),
}


def main() -> None:
    cfg = Config()
    t, truth_state = simulate_robot_motion(cfg)
    true_positions = truth_state[:, :2]

    base = cfg.output_folder
    before_dir = base / "analysis_4_sensor_a1_before"
    after_dir = base / "analysis_4_sensor_a1_after"
    count_dir = base / "analysis_sensor_count_4_to_10"

    for folder in [before_dir, after_dir, count_dir]:
        folder.mkdir(parents=True, exist_ok=True)

    before_rows = generate_geometry_set(
        cfg,
        t,
        true_positions,
        A1_BEFORE_GEOMETRIES_4,
        before_dir,
        title_prefix="A1 before revision",
        seed_base=11_000,
    )
    after_rows = generate_geometry_set(
        cfg,
        t,
        true_positions,
        SENSOR_GEOMETRIES_4,
        after_dir,
        title_prefix="A1 after revision: A1=(0,10)",
        seed_base=12_000,
    )
    generate_before_after_comparison(before_rows, after_rows, base / "analysis_4_sensor_a1_before_after_summary.png")

    count_rows = generate_sensor_count_set(cfg, t, true_positions, count_dir)

    print("Detailed analysis figures generated.")
    print(f"4-sensor A1 before folder: {before_dir.resolve()}")
    print(f"4-sensor A1 after folder:  {after_dir.resolve()}")
    print(f"4-10 sensor folder:        {count_dir.resolve()}")
    print("Best current 4-10 sensor count by mean RMSE: " + best_count_label(count_rows))


def generate_geometry_set(
    cfg: Config,
    t: np.ndarray,
    true_positions: np.ndarray,
    geometries: dict[str, np.ndarray],
    folder: Path,
    title_prefix: str,
    seed_base: int,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []

    for idx, (name, sensors) in enumerate(geometries.items()):
        fixed_result = evaluate_layout(cfg, true_positions, sensors, np.random.default_rng(seed_base + idx))
        mc = monte_carlo_layout(cfg, true_positions, sensors, base_seed=seed_base + 100 * (idx + 1))

        row = {
            "geometry": name,
            "sensor_count": sensors.shape[0],
            "single_run_rmse_m": fixed_result["metrics"]["rmse_position_m"],
            "single_run_mean_error_m": fixed_result["metrics"]["mean_position_error_m"],
            "single_run_max_error_m": fixed_result["metrics"]["max_position_error_m"],
            "mean_nlos_anchor_count": fixed_result["metrics"]["mean_nlos_anchor_count"],
            **mc,
        }
        rows.append(row)

        plot_layout_case(
            cfg,
            t,
            true_positions,
            sensors,
            fixed_result,
            folder / f"{name}_map_and_error.png",
            f"{title_prefix} - {name}",
        )

    write_rows(folder / "summary.csv", rows)
    plot_geometry_summary(rows, folder / "summary_rmse_by_geometry.png", title_prefix)
    return rows


def generate_sensor_count_set(
    cfg: Config,
    t: np.ndarray,
    true_positions: np.ndarray,
    folder: Path,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []

    for count in range(4, SENSOR_POOL_MAX_10.shape[0] + 1):
        sensors = SENSOR_POOL_MAX_10[:count]
        fixed_result = evaluate_layout(cfg, true_positions, sensors, np.random.default_rng(20_000 + count))
        mc = monte_carlo_layout(cfg, true_positions, sensors, base_seed=21_000 + 100 * count)

        row = {
            "sensor_count": count,
            "single_run_rmse_m": fixed_result["metrics"]["rmse_position_m"],
            "single_run_mean_error_m": fixed_result["metrics"]["mean_position_error_m"],
            "single_run_max_error_m": fixed_result["metrics"]["max_position_error_m"],
            "mean_nlos_anchor_count": fixed_result["metrics"]["mean_nlos_anchor_count"],
            **mc,
        }
        rows.append(row)

        plot_layout_case(
            cfg,
            t,
            true_positions,
            sensors,
            fixed_result,
            folder / f"sensor_count_{count:02d}_map_and_error.png",
            f"Current A1=(0,10), {count} sensors",
        )

    write_rows(folder / "summary.csv", rows)
    plot_sensor_count_summary(rows, folder / "summary_sensor_count_effect.png")
    return rows


def plot_layout_case(
    cfg: Config,
    t: np.ndarray,
    true_positions: np.ndarray,
    sensors: np.ndarray,
    result: dict[str, object],
    output_path: Path,
    title: str,
) -> None:
    ekf = result["ekf"]
    measurements = result["measurements"]
    metrics = result["metrics"]
    assert isinstance(ekf, dict)
    assert isinstance(measurements, dict)
    assert isinstance(metrics, dict)

    estimate = ekf["position"]
    error = np.linalg.norm(estimate - true_positions, axis=1)
    nlos_count = measurements["is_nlos"].sum(axis=1)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6), dpi=150)
    draw_environment(axes[0], cfg)
    axes[0].plot(MISSION_POINTS[:, 0], MISSION_POINTS[:, 1], "o--", color="#92400e", linewidth=1.2, label="Mission route")
    axes[0].plot(true_positions[:, 0], true_positions[:, 1], color="black", linewidth=2.0, label="True path")
    axes[0].plot(estimate[:, 0], estimate[:, 1], color="#be123c", linewidth=1.5, label="EKF estimate")
    axes[0].scatter(true_positions[nlos_count > 0, 0], true_positions[nlos_count > 0, 1], s=10, color="#f97316", alpha=0.55, label="NLOS instant")
    axes[0].scatter(sensors[:, 0], sensors[:, 1], s=70, color="#1d4ed8", zorder=8, label="UWB anchors")
    for idx, sensor in enumerate(sensors, start=1):
        axes[0].text(sensor[0] + 0.35, sensor[1] + 0.35, f"A{idx}", fontsize=8, color="#1e3a8a")
    mark_mission_points(axes[0])
    axes[0].legend(loc="upper right", fontsize=7)
    axes[0].set_title("Map, sensors and EKF track")

    axes[1].plot(t, error, color="#047857", linewidth=1.5, label="Position error")
    axes[1].axvspan(10.0, 15.0, color="#f59e0b", alpha=0.18, label="Pickup wait")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("position error [m]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].set_title(
        f"single RMSE={metrics['rmse_position_m']:.3f} m, "
        f"mean NLOS={metrics['mean_nlos_anchor_count']:.2f}"
    )

    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def draw_environment(ax, cfg: Config) -> None:
    zones = [
        ((0.0, 20.0), 25.0, 10.0, "#dbeafe", "Parking Docks"),
        ((0.0, 10.0), 25.0, 10.0, "#fde68a", "Battery Loading / Pickup"),
        ((25.0, 10.0), 25.0, 20.0, "#dcfce7", "Main transit aisle"),
        ((0.0, 0.0), 50.0, 10.0, "#f3f4f6", "Lower service area"),
    ]
    for xy, width, height, color, label in zones:
        ax.add_patch(Rectangle(xy, width, height, facecolor=color, edgecolor="white", linewidth=0.8, alpha=0.55, zorder=1))
        ax.text(xy[0] + width / 2, xy[1] + height / 2, label, ha="center", va="center", fontsize=7, color="#111827", zorder=2)

    for xmin, ymin, xmax, ymax in inflated_obstacles(cfg):
        ax.add_patch(Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, facecolor="#ef4444", edgecolor="#991b1b", alpha=0.25, linewidth=0.7, zorder=4))

    ax.add_patch(Rectangle((0, 0), cfg.factory_size[0], cfg.factory_size[1], fill=False, edgecolor="#111827", linewidth=2.0, zorder=8))
    ax.set_xlim(-1, cfg.factory_size[0] + 1)
    ax.set_ylim(-1, cfg.factory_size[1] + 1)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def mark_mission_points(ax) -> None:
    points = [
        (PARKING_START_INDEX, "Start", "#2563eb", "o"),
        (PICKUP_INDEX, "Pickup", "#f59e0b", "D"),
        (GATE2_INDEX, "Gate-2", "#16a34a", "s"),
        (DROPOFF_INDEX, "Drop-off", "#dc2626", "*"),
    ]
    for idx, label, color, marker in points:
        p = MISSION_POINTS[idx]
        ax.scatter([p[0]], [p[1]], s=95, color=color, marker=marker, zorder=9)
        ax.text(p[0] + 0.35, p[1] + 0.35, label, fontsize=7, color=color, zorder=9)


def plot_geometry_summary(rows: list[dict[str, object]], output_path: Path, title: str) -> None:
    names = [str(row["geometry"]).replace("_", "\n") for row in rows]
    means = np.array([float(row["mean_rmse_m"]) for row in rows])
    stds = np.array([float(row["std_rmse_m"]) for row in rows])
    singles = np.array([float(row["single_run_rmse_m"]) for row in rows])

    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    x = np.arange(len(rows))
    ax.bar(x, means, yerr=stds, capsize=4, color="#2563eb", alpha=0.78, label="Monte Carlo mean RMSE")
    ax.plot(x, singles, "o-", color="#dc2626", linewidth=1.4, label="Single run RMSE")
    ax.set_xticks(x, names)
    ax.set_ylabel("RMSE [m]")
    ax.set_title(title)
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def generate_before_after_comparison(before_rows: list[dict[str, object]], after_rows: list[dict[str, object]], output_path: Path) -> None:
    names = [str(row["geometry"]).replace("_", "\n") for row in before_rows]
    before = np.array([float(row["mean_rmse_m"]) for row in before_rows])
    after = np.array([float(row["mean_rmse_m"]) for row in after_rows])

    fig, ax = plt.subplots(figsize=(8, 5), dpi=150)
    x = np.arange(len(names))
    width = 0.36
    ax.bar(x - width / 2, before, width, label="A1 before", color="#64748b")
    ax.bar(x + width / 2, after, width, label="A1 after (0,10)", color="#2563eb")
    ax.set_xticks(x, names)
    ax.set_ylabel("Monte Carlo mean RMSE [m]")
    ax.set_title("4-sensor A1 before/after comparison")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def plot_sensor_count_summary(rows: list[dict[str, object]], output_path: Path) -> None:
    counts = np.array([int(row["sensor_count"]) for row in rows])
    mean_rmse = np.array([float(row["mean_rmse_m"]) for row in rows])
    std_rmse = np.array([float(row["std_rmse_m"]) for row in rows])
    condition = np.array([float(row["mean_condition_number"]) for row in rows])
    nlos = np.array([float(row["mean_nlos_anchor_count"]) for row in rows])

    fig, axes = plt.subplots(3, 1, figsize=(9, 10), dpi=150, sharex=True)
    axes[0].errorbar(counts, mean_rmse, yerr=std_rmse, marker="o", linewidth=1.6, capsize=4, color="#2563eb")
    axes[0].set_ylabel("Mean RMSE [m]")
    axes[0].set_title("Effect of adding sensors, current A1=(0,10)")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(counts, condition, "o-", color="#7c3aed", linewidth=1.6)
    axes[1].set_ylabel("Mean cond(H^T H)")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(counts, nlos, "o-", color="#f97316", linewidth=1.6)
    axes[2].set_ylabel("Single-run mean NLOS anchors")
    axes[2].set_xlabel("Sensor count")
    axes[2].set_xticks(counts)
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def best_count_label(rows: list[dict[str, object]]) -> str:
    best = min(rows, key=lambda row: float(row["mean_rmse_m"]))
    return f"{best['sensor_count']} sensors, mean RMSE={float(best['mean_rmse_m']):.3f} m"


if __name__ == "__main__":
    main()

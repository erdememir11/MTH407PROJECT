"""
MTH407 Donem Projesi
Fabrika ortaminda batarya yerlestirme robotu takibi

Model:
- 50 m x 30 m fabrika alani
- Parking Docks bolgesinden baslayan, Battery Loading / Pickup noktasinda
  5 saniye bekleyerek bataryayi alan ve drop-off hedefine en az yon
  degisimiyle giden gorev
- UWB anchor antenleri ile TDOA olcumu
- LSE ilklendirme + EKF takip
- 4 sensor icin 3 farkli geometri analizi
- 4-10 sensor sayisi icin performans analizi

Calistirma:
    python agv_tdoa_ekf.py
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


@dataclass(frozen=True)
class Config:
    output_folder: Path = Path("outputs")
    c: float = 299_792_458.0
    dt: float = 0.5
    nominal_speed: float = 1.0
    pickup_wait_time: float = 5.0
    factory_size: tuple[float, float] = (50.0, 30.0)
    robot_radius: float = 0.35
    safety_margin: float = 0.25
    wall_thickness: float = 0.20
    sigma_accel: float = 0.35
    reference_sensor: int = 0
    lse_iterations: int = 25
    monte_carlo_runs: int = 40

    @property
    def d_safe(self) -> float:
        return self.robot_radius + self.safety_margin

    @property
    def sigma_toa_los(self) -> float:
        return 0.25 / self.c

    @property
    def sigma_toa_nlos(self) -> float:
        return 1.50 / self.c


MISSION_POINTS = np.array(
    [
        [5.0, 25.0],   # Parking Dock start
        [5.0, 15.0],   # Battery pickup point
        [25.0, 18.0],  # Gate-2 / main corridor entry
        [45.0, 10.0],  # Drop-off
    ],
    dtype=float,
)

PARKING_START_INDEX = 0
PICKUP_INDEX = 1
GATE2_INDEX = 2
DROPOFF_INDEX = 3


SENSOR_GEOMETRIES_4 = {
    "G1_corner_coverage": np.array(
        [
            [1.0, 1.0],
            [49.0, 1.0],
            [49.0, 29.0],
            [1.0, 29.0],
        ],
        dtype=float,
    ),
    "G2_task_oriented": np.array(
        [
            [3.0, 27.5],
            [24.0, 27.5],
            [28.0, 15.5],
            [48.0, 11.0],
        ],
        dtype=float,
    ),
    "G3_poor_same_wall": np.array(
        [
            [2.0, 2.0],
            [17.0, 2.0],
            [33.0, 2.0],
            [48.0, 2.0],
        ],
        dtype=float,
    ),
}


SENSOR_POOL_MAX_10 = np.array(
    [
        [1.0, 1.0],
        [49.0, 1.0],
        [49.0, 29.0],
        [1.0, 29.0],
        [25.0, 29.0],
        [49.0, 15.0],
        [1.0, 15.0],
        [25.0, 1.0],
        [25.0, 18.0],
        [45.0, 10.0],
    ],
    dtype=float,
)


def inflated_obstacles(cfg: Config) -> list[tuple[float, float, float, float]]:
    """Return inflated forbidden rectangles as (xmin, ymin, xmax, ymax)."""
    half = cfg.wall_thickness / 2.0 + cfg.d_safe
    obs: list[tuple[float, float, float, float]] = []

    # B1: y=20, 0<=x<=25 with Gate-1 around x=5.
    # This separates Parking Docks from Battery Loading / Pickup.
    obs.append((0.0, 20.0 - half, 4.0, 20.0 + half))
    obs.append((6.0, 20.0 - half, 25.0, 20.0 + half))

    # B2: x=25 with Gate-2 y in [16.6, 19.0].
    # Lower part is extended down to B3, removing the old ineffective gap.
    obs.append((25.0 - half, 10.0, 25.0 + half, 16.6))
    obs.append((25.0 - half, 19.0, 25.0 + half, 30.0))

    # B3: y=10, 0<=x<=40. Drop-off approach remains open for x>40.
    obs.append((0.0, 10.0 - half, 40.0, 10.0 + half))

    # B4: x=40, 0<=y<=10.
    obs.append((40.0 - half, 0.0, 40.0 + half, 10.0))

    return obs


def simulate_robot_motion(cfg: Config) -> tuple[np.ndarray, np.ndarray]:
    obs = inflated_obstacles(cfg)
    samples: list[np.ndarray] = []
    current = MISSION_POINTS[PARKING_START_INDEX].copy()
    samples.append(np.array([current[0], current[1], 0.0, 0.0], dtype=float))

    # 1) Parking Dock -> Pickup: vertical, constant-speed motion through Gate-1.
    current = append_motion_segment(samples, current, MISSION_POINTS[PICKUP_INDEX], cfg, obs)

    # 2) Pickup operation: battery loading is represented by a 5-second stop.
    wait_steps = int(round(cfg.pickup_wait_time / cfg.dt))
    for _ in range(wait_steps):
        samples.append(np.array([current[0], current[1], 0.0, 0.0], dtype=float))

    # 3) Pickup -> Gate-2: constant speed, entering the main corridor through the opening.
    current = append_motion_segment(samples, current, MISSION_POINTS[GATE2_INDEX], cfg, obs)

    # 4) Gate-2 -> Drop-off: minimum direction-change path in the main corridor.
    append_motion_segment(samples, current, MISSION_POINTS[DROPOFF_INDEX], cfg, obs)

    state = np.vstack(samples)
    t = np.arange(state.shape[0]) * cfg.dt
    return t, state


def append_motion_segment(
    samples: list[np.ndarray],
    start: np.ndarray,
    target: np.ndarray,
    cfg: Config,
    obstacles: list[tuple[float, float, float, float]],
) -> np.ndarray:
    current = start.copy()

    while True:
        delta = target - current
        distance = float(np.linalg.norm(delta))
        if distance <= 1e-9:
            return target.copy()

        step_length = min(cfg.nominal_speed * cfg.dt, distance)
        direction = delta / distance
        next_position = current + direction * step_length
        velocity = (next_position - current) / cfg.dt

        if not inside_map(next_position, cfg) or collision_point(next_position, obstacles):
            raise RuntimeError(
                f"Mission segment collision: start={start}, target={target}, candidate={next_position}."
            )

        samples.append(np.array([next_position[0], next_position[1], velocity[0], velocity[1]], dtype=float))
        current = next_position

        if np.linalg.norm(target - current) <= 1e-9:
            return target.copy()


def inside_map(p: np.ndarray, cfg: Config) -> bool:
    return 0.0 <= p[0] <= cfg.factory_size[0] and 0.0 <= p[1] <= cfg.factory_size[1]


def collision_point(p: np.ndarray, obstacles: list[tuple[float, float, float, float]]) -> bool:
    return any(xmin <= p[0] <= xmax and ymin <= p[1] <= ymax for xmin, ymin, xmax, ymax in obstacles)


def generate_tdoa_measurements(
    positions: np.ndarray,
    cfg: Config,
    sensors: np.ndarray,
    rng: np.random.Generator,
) -> dict[str, np.ndarray]:
    m = sensors.shape[0]
    ref = cfg.reference_sensor
    other = [i for i in range(m) if i != ref]
    k_count = positions.shape[0]
    obstacles = inflated_obstacles(cfg)

    z_range_diff = np.zeros((k_count, m - 1), dtype=float)
    z_time_diff = np.zeros((k_count, m - 1), dtype=float)
    r_mats = np.zeros((k_count, m - 1, m - 1), dtype=float)
    is_nlos = np.zeros((k_count, m), dtype=bool)

    for k, p in enumerate(positions):
        distances = np.linalg.norm(sensors - p, axis=1)
        sigma_toa = np.full(m, cfg.sigma_toa_los)

        for i, anchor in enumerate(sensors):
            if line_intersects_any_obstacle(p, anchor, obstacles):
                sigma_toa[i] = cfg.sigma_toa_nlos
                is_nlos[k, i] = True

        noisy_toa = distances / cfg.c + sigma_toa * rng.normal(size=m)
        tdoa = noisy_toa[other] - noisy_toa[ref]

        z_time_diff[k] = tdoa
        z_range_diff[k] = cfg.c * tdoa
        range_var = cfg.c**2 * (sigma_toa[other] ** 2 + sigma_toa[ref] ** 2)
        r_mats[k] = np.diag(range_var)

    return {"z_range_diff": z_range_diff, "z_time_diff": z_time_diff, "r": r_mats, "is_nlos": is_nlos}


def initialize_with_lse(
    measurements: dict[str, np.ndarray],
    cfg: Config,
    sensors: np.ndarray,
) -> dict[str, np.ndarray]:
    count = min(8, measurements["z_range_diff"].shape[0])
    p_hat = np.zeros((count, 2), dtype=float)
    p_cov = np.zeros((count, 2, 2), dtype=float)

    for k in range(count):
        p_hat[k], p_cov[k] = estimate_position_lse(
            measurements["z_range_diff"][k],
            measurements["r"][k],
            cfg,
            sensors,
        )

    times = np.arange(count) * cfg.dt
    vx = np.polyfit(times, p_hat[:, 0], 1)[0]
    vy = np.polyfit(times, p_hat[:, 1], 1)[0]

    x0 = np.array([p_hat[0, 0], p_hat[0, 1], vx, vy], dtype=float)
    p0 = np.zeros((4, 4), dtype=float)
    p0[:2, :2] = p_cov[0] + 0.50 * np.eye(2)
    p0[2:, 2:] = np.diag([0.8**2, 0.8**2])
    return {"x0": x0, "p0": p0, "lse_positions": p_hat}


def estimate_position_lse(
    z: np.ndarray,
    r_mat: np.ndarray,
    cfg: Config,
    sensors: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    p = sensors.mean(axis=0).copy()
    w = np.linalg.inv(r_mat)

    for _ in range(cfg.lse_iterations):
        h, h_pos = tdoa_measurement_model(p, sensors, cfg.reference_sensor)
        residual = z - h
        normal = h_pos.T @ w @ h_pos + 1e-6 * np.eye(2)
        step = np.linalg.solve(normal, h_pos.T @ w @ residual)
        p += step
        p[0] = np.clip(p[0], 0.2, cfg.factory_size[0] - 0.2)
        p[1] = np.clip(p[1], 0.2, cfg.factory_size[1] - 0.2)
        if np.linalg.norm(step) < 1e-5:
            break

    _, h_pos = tdoa_measurement_model(p, sensors, cfg.reference_sensor)
    cov = np.linalg.inv(h_pos.T @ w @ h_pos + 1e-6 * np.eye(2))
    return p, cov


def run_ekf_tracker(
    measurements: dict[str, np.ndarray],
    init: dict[str, np.ndarray],
    cfg: Config,
    sensors: np.ndarray,
) -> dict[str, np.ndarray]:
    k_count = measurements["z_range_diff"].shape[0]
    state = np.zeros((k_count, 4), dtype=float)
    cov = np.zeros((k_count, 4, 4), dtype=float)
    innovation_norm = np.zeros(k_count, dtype=float)

    state[0] = init["x0"]
    cov[0] = init["p0"]

    dt = cfg.dt
    f = np.array(
        [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    g = np.array([[0.5 * dt**2, 0.0], [0.0, 0.5 * dt**2], [dt, 0.0], [0.0, dt]])
    q = cfg.sigma_accel**2 * (g @ g.T)
    identity = np.eye(4)

    for k in range(1, k_count):
        x_pred = f @ state[k - 1]
        p_pred = f @ cov[k - 1] @ f.T + q

        h, h_pos = tdoa_measurement_model(x_pred[:2], sensors, cfg.reference_sensor)
        h_jac = np.hstack((h_pos, np.zeros((h_pos.shape[0], 2))))
        r_k = measurements["r"][k]
        y = measurements["z_range_diff"][k] - h
        s_mat = h_jac @ p_pred @ h_jac.T + r_k
        k_gain = p_pred @ h_jac.T @ np.linalg.inv(s_mat)

        x_upd = x_pred + k_gain @ y
        p_upd = (identity - k_gain @ h_jac) @ p_pred @ (identity - k_gain @ h_jac).T
        p_upd += k_gain @ r_k @ k_gain.T

        x_upd[0] = np.clip(x_upd[0], 0.0, cfg.factory_size[0])
        x_upd[1] = np.clip(x_upd[1], 0.0, cfg.factory_size[1])

        state[k] = x_upd
        cov[k] = p_upd
        innovation_norm[k] = math.sqrt(float(y.T @ np.linalg.inv(s_mat) @ y))

    return {"state": state, "position": state[:, :2], "p": cov, "innovation_norm": innovation_norm}


def tdoa_measurement_model(
    position: np.ndarray,
    sensors: np.ndarray,
    reference_sensor: int,
) -> tuple[np.ndarray, np.ndarray]:
    p = np.asarray(position, dtype=float)
    m = sensors.shape[0]
    other = [i for i in range(m) if i != reference_sensor]

    diff = p - sensors
    distances = np.maximum(np.linalg.norm(diff, axis=1), 1e-6)

    h = distances[other] - distances[reference_sensor]
    h_pos = np.zeros((m - 1, 2), dtype=float)
    for row, i in enumerate(other):
        h_pos[row] = diff[i] / distances[i] - diff[reference_sensor] / distances[reference_sensor]
    return h, h_pos


def compute_metrics(true_positions: np.ndarray, estimated_positions: np.ndarray) -> dict[str, float]:
    error = np.linalg.norm(estimated_positions - true_positions, axis=1)
    return {
        "rmse_position_m": float(np.sqrt(np.mean(error**2))),
        "mean_position_error_m": float(np.mean(error)),
        "max_position_error_m": float(np.max(error)),
    }


def evaluate_layout(
    cfg: Config,
    true_positions: np.ndarray,
    sensors: np.ndarray,
    rng: np.random.Generator,
) -> dict[str, object]:
    measurements = generate_tdoa_measurements(true_positions, cfg, sensors, rng)
    init = initialize_with_lse(measurements, cfg, sensors)
    ekf = run_ekf_tracker(measurements, init, cfg, sensors)
    metrics = compute_metrics(true_positions, ekf["position"])
    metrics["mean_nlos_anchor_count"] = float(measurements["is_nlos"].sum(axis=1).mean())
    metrics["mean_condition_number"] = mean_condition_number(true_positions, sensors, cfg)
    return {"measurements": measurements, "init": init, "ekf": ekf, "metrics": metrics}


def monte_carlo_layout(
    cfg: Config,
    true_positions: np.ndarray,
    sensors: np.ndarray,
    base_seed: int,
) -> dict[str, float]:
    rmse = np.zeros(cfg.monte_carlo_runs, dtype=float)
    mean_err = np.zeros(cfg.monte_carlo_runs, dtype=float)
    max_err = np.zeros(cfg.monte_carlo_runs, dtype=float)

    for run in range(cfg.monte_carlo_runs):
        rng = np.random.default_rng(base_seed + run)
        result = evaluate_layout(cfg, true_positions, sensors, rng)
        rmse[run] = result["metrics"]["rmse_position_m"]
        mean_err[run] = result["metrics"]["mean_position_error_m"]
        max_err[run] = result["metrics"]["max_position_error_m"]

    return {
        "mean_rmse_m": float(rmse.mean()),
        "std_rmse_m": float(rmse.std(ddof=1)),
        "mean_error_m": float(mean_err.mean()),
        "max_error_m": float(max_err.mean()),
        "mean_condition_number": mean_condition_number(true_positions, sensors, cfg),
    }


def analyze_four_sensor_geometries(cfg: Config, true_positions: np.ndarray) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    for idx, (name, sensors) in enumerate(SENSOR_GEOMETRIES_4.items()):
        metrics = monte_carlo_layout(cfg, true_positions, sensors, base_seed=5000 + 100 * idx)
        rows.append({"geometry": name, "sensor_count": 4, **metrics})
    return rows


def analyze_sensor_count(cfg: Config, true_positions: np.ndarray) -> list[dict[str, float | int]]:
    rows: list[dict[str, float | int]] = []
    for count in range(4, len(SENSOR_POOL_MAX_10) + 1):
        sensors = SENSOR_POOL_MAX_10[:count]
        metrics = monte_carlo_layout(cfg, true_positions, sensors, base_seed=8000 + 100 * count)
        rows.append({"sensor_count": count, **metrics})
    return rows


def mean_condition_number(true_positions: np.ndarray, sensors: np.ndarray, cfg: Config) -> float:
    values = []
    for p in true_positions:
        _, h_pos = tdoa_measurement_model(p, sensors, cfg.reference_sensor)
        values.append(np.linalg.cond(h_pos.T @ h_pos + 1e-9 * np.eye(2)))
    return float(np.mean(values))


def line_intersects_any_obstacle(
    p1: np.ndarray,
    p2: np.ndarray,
    obstacles: list[tuple[float, float, float, float]],
) -> bool:
    return any(segment_intersects_rect(p1, p2, rect) for rect in obstacles)


def segment_intersects_rect(p1: np.ndarray, p2: np.ndarray, rect: tuple[float, float, float, float]) -> bool:
    xmin, ymin, xmax, ymax = rect
    if xmin <= p1[0] <= xmax and ymin <= p1[1] <= ymax:
        return True
    if xmin <= p2[0] <= xmax and ymin <= p2[1] <= ymax:
        return True

    corners = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    return any(segments_intersect(p1, p2, corners[a], corners[b]) for a, b in edges)


def segments_intersect(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


def ccw(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    return bool((c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0]))


def save_csv(path: Path, rows: list[dict[str, object]]) -> None:
    if not rows:
        return
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def save_tracking_outputs(
    cfg: Config,
    t: np.ndarray,
    true_positions: np.ndarray,
    main_result: dict[str, object],
    geometry_rows: list[dict[str, object]],
    count_rows: list[dict[str, object]],
) -> None:
    cfg.output_folder.mkdir(exist_ok=True)
    ekf = main_result["ekf"]
    measurements = main_result["measurements"]
    assert isinstance(ekf, dict)
    assert isinstance(measurements, dict)

    errors = np.linalg.norm(ekf["position"] - true_positions, axis=1)
    rows = []
    for k in range(len(t)):
        rows.append(
            {
                "time_s": f"{t[k]:.6f}",
                "true_x_m": f"{true_positions[k, 0]:.6f}",
                "true_y_m": f"{true_positions[k, 1]:.6f}",
                "estimated_x_m": f"{ekf['position'][k, 0]:.6f}",
                "estimated_y_m": f"{ekf['position'][k, 1]:.6f}",
                "position_error_m": f"{errors[k]:.6f}",
                "nlos_anchor_count": int(measurements["is_nlos"][k].sum()),
            }
        )
    save_csv(cfg.output_folder / "tracking_results.csv", rows)

    metrics = main_result["metrics"]
    assert isinstance(metrics, dict)
    save_csv(cfg.output_folder / "main_metrics.csv", [{k: f"{v:.6f}" for k, v in metrics.items()}])
    save_csv(cfg.output_folder / "four_sensor_geometry_analysis.csv", geometry_rows)
    save_csv(cfg.output_folder / "sensor_count_analysis.csv", count_rows)


def plot_environment(
    cfg: Config,
    t: np.ndarray,
    true_positions: np.ndarray,
    main_result: dict[str, object],
    sensors: np.ndarray,
) -> None:
    ekf = main_result["ekf"]
    measurements = main_result["measurements"]
    assert isinstance(ekf, dict)
    assert isinstance(measurements, dict)

    fig, ax = plt.subplots(figsize=(11, 7), dpi=150)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, cfg.factory_size[0])
    ax.set_ylim(0, cfg.factory_size[1])
    ax.grid(True, alpha=0.25)

    # Semantic zones.
    zones = [
        ((0, 20), 25, 10, "#dbeafe", "Parking Docks"),
        ((0, 10), 25, 10, "#fde68a", "Battery Loading / Pickup"),
        ((25, 10), 25, 20, "#dcfce7", "Main transit aisle"),
    ]
    for xy, width, height, color, label in zones:
        ax.add_patch(Rectangle(xy, width, height, facecolor=color, edgecolor="none", alpha=0.45, label=label))

    for rect in inflated_obstacles(cfg):
        xmin, ymin, xmax, ymax = rect
        ax.add_patch(
            Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                facecolor="#ef4444",
                edgecolor="#7f1d1d",
                alpha=0.35,
                linewidth=1.0,
            )
        )

    nlos_count = measurements["is_nlos"].sum(axis=1)
    ax.scatter(
        true_positions[nlos_count > 0, 0],
        true_positions[nlos_count > 0, 1],
        s=12,
        color="#f97316",
        alpha=0.65,
        label="NLOS measurement instant",
    )
    ax.plot(
        MISSION_POINTS[:, 0],
        MISSION_POINTS[:, 1],
        "o--",
        color="#92400e",
        linewidth=1.0,
        label="Mission route",
    )
    ax.plot(true_positions[:, 0], true_positions[:, 1], "k-", linewidth=2.0, label="True robot path")
    ax.plot(ekf["position"][:, 0], ekf["position"][:, 1], color="#be123c", linewidth=1.7, label="EKF estimate")

    ax.scatter(sensors[:, 0], sensors[:, 1], s=65, color="#1d4ed8", label="UWB anchors")
    for idx, sensor in enumerate(sensors, start=1):
        ax.text(sensor[0] + 0.35, sensor[1] + 0.35, f"A{idx}", color="#1e3a8a", fontsize=8)

    ax.scatter(
        MISSION_POINTS[PARKING_START_INDEX, 0],
        MISSION_POINTS[PARKING_START_INDEX, 1],
        s=80,
        color="#2563eb",
        marker="o",
        label="Start / Parking",
    )
    ax.scatter(
        MISSION_POINTS[PICKUP_INDEX, 0],
        MISSION_POINTS[PICKUP_INDEX, 1],
        s=90,
        color="#f59e0b",
        marker="D",
        label="Pickup",
    )
    ax.scatter(
        MISSION_POINTS[DROPOFF_INDEX, 0],
        MISSION_POINTS[DROPOFF_INDEX, 1],
        s=120,
        color="#dc2626",
        marker="*",
        label="Drop-off",
    )
    ax.text(
        MISSION_POINTS[PARKING_START_INDEX, 0] + 0.45,
        MISSION_POINTS[PARKING_START_INDEX, 1] + 0.55,
        "Start\nParking",
        fontsize=8,
        color="#1e3a8a",
    )
    ax.text(
        MISSION_POINTS[PICKUP_INDEX, 0] + 0.45,
        MISSION_POINTS[PICKUP_INDEX, 1] + 0.55,
        "Pickup\n5 s wait",
        fontsize=8,
        color="#92400e",
    )
    ax.text(
        MISSION_POINTS[DROPOFF_INDEX, 0] + 0.45,
        MISSION_POINTS[DROPOFF_INDEX, 1] - 1.25,
        "Drop-off",
        fontsize=8,
        color="#991b1b",
    )
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Parking-to-pickup battery robot tracking with UWB/TDOA")
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5), fontsize=8)
    fig.tight_layout()
    fig.savefig(cfg.output_folder / "battery_robot_tracking_map.png")
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(10, 8), dpi=150, sharex=True)
    err = np.linalg.norm(ekf["position"] - true_positions, axis=1)
    axes[0].plot(t, true_positions[:, 0], "k-", label="True")
    axes[0].plot(t, ekf["position"][:, 0], "r--", label="Estimate")
    axes[0].set_ylabel("x [m]")
    axes[0].legend()
    axes[1].plot(t, true_positions[:, 1], "k-")
    axes[1].plot(t, ekf["position"][:, 1], "r--")
    axes[1].set_ylabel("y [m]")
    axes[2].plot(t, err, color="#047857")
    axes[2].set_ylabel("Position error [m]")
    axes[2].set_xlabel("time [s]")
    axes[2].set_title(f"RMSE = {math.sqrt(float(np.mean(err**2))):.2f} m")
    for axis in axes:
        axis.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(cfg.output_folder / "tracking_time_series.png")
    plt.close(fig)


def plot_analysis(
    cfg: Config,
    geometry_rows: list[dict[str, object]],
    count_rows: list[dict[str, object]],
) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), dpi=150)

    names = [str(row["geometry"]).replace("_", "\n") for row in geometry_rows]
    rmse = np.array([float(row["mean_rmse_m"]) for row in geometry_rows])
    std = np.array([float(row["std_rmse_m"]) for row in geometry_rows])
    axes[0].bar(np.arange(len(names)), rmse, yerr=std, color=["#2563eb", "#059669", "#dc2626"], capsize=4)
    axes[0].set_xticks(np.arange(len(names)), names)
    axes[0].set_ylabel("Mean RMSE [m]")
    axes[0].set_title("4-sensor geometry comparison")
    axes[0].grid(True, axis="y", alpha=0.3)

    counts = np.array([int(row["sensor_count"]) for row in count_rows])
    count_rmse = np.array([float(row["mean_rmse_m"]) for row in count_rows])
    count_std = np.array([float(row["std_rmse_m"]) for row in count_rows])
    axes[1].errorbar(counts, count_rmse, yerr=count_std, marker="o", linewidth=1.7, capsize=4, color="#7c3aed")
    axes[1].set_xticks(counts)
    axes[1].set_xlabel("Sensor count")
    axes[1].set_ylabel("Mean RMSE [m]")
    axes[1].set_title("Sensor count analysis, 4 to 10 anchors")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(cfg.output_folder / "geometry_and_sensor_count_analysis.png")
    plt.close(fig)


def main() -> None:
    cfg = Config()
    cfg.output_folder.mkdir(exist_ok=True)

    t, truth_state = simulate_robot_motion(cfg)
    true_positions = truth_state[:, :2]

    main_sensors = SENSOR_GEOMETRIES_4["G1_corner_coverage"]
    main_result = evaluate_layout(cfg, true_positions, main_sensors, np.random.default_rng(407))
    geometry_rows = analyze_four_sensor_geometries(cfg, true_positions)
    count_rows = analyze_sensor_count(cfg, true_positions)

    save_tracking_outputs(cfg, t, true_positions, main_result, geometry_rows, count_rows)
    plot_environment(cfg, t, true_positions, main_result, main_sensors)
    plot_analysis(cfg, geometry_rows, count_rows)

    metrics = main_result["metrics"]
    assert isinstance(metrics, dict)
    print("Parking-to-pickup battery robot UWB/TDOA-EKF simulation completed.")
    print(f"Main layout RMSE: {metrics['rmse_position_m']:.3f} m")
    print(f"Mean NLOS anchor count: {metrics['mean_nlos_anchor_count']:.2f} / {main_sensors.shape[0]}")
    print("4-sensor geometry analysis:")
    for row in geometry_rows:
        print(f"  {row['geometry']}: RMSE={float(row['mean_rmse_m']):.3f} m")
    print("Sensor count analysis:")
    for row in count_rows:
        print(f"  {row['sensor_count']} sensors: RMSE={float(row['mean_rmse_m']):.3f} m")
    print(f"Outputs: {cfg.output_folder.resolve()}")


if __name__ == "__main__":
    main()

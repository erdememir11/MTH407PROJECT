"""
MTH407 Donem Projesi
Akilli Fabrikalarda Otonom Robot (AGV) Takibi

2B fabrika ortaminda UWB anchor antenleri ile TDOA olcumu uretilir.
Ilk konum LSE (Gauss-Newton) ile bulunur, sonraki takip EKF ile yapilir.

Calistirma:
    python agv_tdoa_ekf.py
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


@dataclass(frozen=True)
class Config:
    output_folder: Path = Path("outputs")
    c: float = 299_792_458.0
    dt: float = 0.2
    n_steps: int = 260
    factory_size: tuple[float, float] = (42.0, 26.0)
    sigma_accel: float = 0.45
    reference_sensor: int = 0
    lse_iterations: int = 20
    geometry_monte_carlo_runs: int = 30

    @property
    def sigma_toa_los(self) -> float:
        return 0.20 / self.c

    @property
    def sigma_toa_nlos(self) -> float:
        return 1.20 / self.c


SENSORS_GOOD = np.array(
    [
        [1.5, 1.5],
        [40.0, 1.2],
        [41.0, 24.5],
        [1.2, 24.0],
        [20.5, 3.2],
        [31.5, 17.5],
    ],
    dtype=float,
)

SENSORS_POOR = np.array(
    [
        [2.0, 1.0],
        [9.0, 1.2],
        [16.0, 0.9],
        [23.0, 1.1],
        [30.0, 0.8],
        [38.0, 1.0],
    ],
    dtype=float,
)

OBSTACLES = np.array(
    [
        [8.0, 6.0, 5.5, 4.0],
        [18.0, 10.0, 7.0, 3.5],
        [29.0, 5.0, 4.0, 8.0],
        [12.0, 18.0, 10.0, 3.0],
    ],
    dtype=float,
)


def simulate_agv_trajectory(cfg: Config) -> tuple[np.ndarray, np.ndarray]:
    t = np.arange(cfg.n_steps) * cfg.dt
    state = np.zeros((cfg.n_steps, 4), dtype=float)
    state[0] = [4.0, 4.0, 1.05, 0.55]

    for k in range(1, cfg.n_steps):
        tk = t[k]
        ax = 0.22 * np.sin(0.23 * tk) - 0.08 * np.sin(0.71 * tk)
        ay = 0.18 * np.cos(0.19 * tk) + 0.06 * np.sin(0.53 * tk)

        state[k, 2:4] = state[k - 1, 2:4] + cfg.dt * np.array([ax, ay])
        speed = np.linalg.norm(state[k, 2:4])
        if speed > 1.8:
            state[k, 2:4] *= 1.8 / speed

        state[k, 0:2] = state[k - 1, 0:2] + cfg.dt * state[k, 2:4]

        margin = 1.0
        if state[k, 0] < margin or state[k, 0] > cfg.factory_size[0] - margin:
            state[k, 2] *= -0.85
            state[k, 0] = np.clip(state[k, 0], margin, cfg.factory_size[0] - margin)
        if state[k, 1] < margin or state[k, 1] > cfg.factory_size[1] - margin:
            state[k, 3] *= -0.85
            state[k, 1] = np.clip(state[k, 1], margin, cfg.factory_size[1] - margin)

    return t, state


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

    z_range_diff = np.zeros((k_count, m - 1), dtype=float)
    z_time_diff = np.zeros((k_count, m - 1), dtype=float)
    r_mats = np.zeros((k_count, m - 1, m - 1), dtype=float)
    is_nlos = np.zeros((k_count, m), dtype=bool)

    for k, p in enumerate(positions):
        distances = np.linalg.norm(sensors - p, axis=1)
        sigma_toa = np.full(m, cfg.sigma_toa_los)

        for i, anchor in enumerate(sensors):
            if line_intersects_any_obstacle(p, anchor, OBSTACLES):
                sigma_toa[i] = cfg.sigma_toa_nlos
                is_nlos[k, i] = True

        noisy_toa = distances / cfg.c + sigma_toa * rng.normal(size=m)
        tdoa = noisy_toa[other] - noisy_toa[ref]

        z_time_diff[k] = tdoa
        z_range_diff[k] = cfg.c * tdoa
        range_var = cfg.c**2 * (sigma_toa[other] ** 2 + sigma_toa[ref] ** 2)
        r_mats[k] = np.diag(range_var)

    return {
        "z_range_diff": z_range_diff,
        "z_time_diff": z_time_diff,
        "r": r_mats,
        "is_nlos": is_nlos,
    }


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
    p0[:2, :2] = p_cov[0] + 0.25 * np.eye(2)
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
        [
            [1.0, 0.0, dt, 0.0],
            [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )
    g = np.array(
        [
            [0.5 * dt**2, 0.0],
            [0.0, 0.5 * dt**2],
            [dt, 0.0],
            [0.0, dt],
        ]
    )
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
        innovation_norm[k] = float(np.sqrt(y.T @ np.linalg.inv(s_mat) @ y))

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


def line_intersects_any_obstacle(p1: np.ndarray, p2: np.ndarray, obstacles: np.ndarray) -> bool:
    return any(segment_intersects_rect(p1, p2, rect) for rect in obstacles)


def segment_intersects_rect(p1: np.ndarray, p2: np.ndarray, rect: np.ndarray) -> bool:
    x, y, w, h = rect
    xmin, xmax = x, x + w
    ymin, ymax = y, y + h

    if point_in_rect(p1, rect) or point_in_rect(p2, rect):
        return True

    corners = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    return any(segments_intersect(p1, p2, corners[a], corners[b]) for a, b in edges)


def point_in_rect(p: np.ndarray, rect: np.ndarray) -> bool:
    return bool(rect[0] <= p[0] <= rect[0] + rect[2] and rect[1] <= p[1] <= rect[1] + rect[3])


def segments_intersect(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


def ccw(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    return bool((c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0]))


def compute_metrics(true_positions: np.ndarray, estimated_positions: np.ndarray) -> dict[str, float]:
    error = np.linalg.norm(estimated_positions - true_positions, axis=1)
    return {
        "rmse_position_m": float(np.sqrt(np.mean(error**2))),
        "mean_position_error_m": float(np.mean(error)),
        "max_position_error_m": float(np.max(error)),
    }


def analyze_sensor_geometry(
    cfg: Config,
    true_positions: np.ndarray,
    rng: np.random.Generator,
) -> dict[str, np.ndarray]:
    names = np.array(["Iyi geometri", "Zayif geometri"], dtype=object)
    sensor_sets = [SENSORS_GOOD, SENSORS_POOR]
    rmse = np.zeros((2, cfg.geometry_monte_carlo_runs), dtype=float)
    mean_condition = np.zeros(2, dtype=float)

    for s_idx, sensors in enumerate(sensor_sets):
        cond_series = []
        for p in true_positions:
            _, h_pos = tdoa_measurement_model(p, sensors, cfg.reference_sensor)
            cond_series.append(np.linalg.cond(h_pos.T @ h_pos + 1e-9 * np.eye(2)))
        mean_condition[s_idx] = float(np.mean(cond_series))

        for run in range(cfg.geometry_monte_carlo_runs):
            meas = generate_tdoa_measurements(true_positions, cfg, sensors, rng)
            init = initialize_with_lse(meas, cfg, sensors)
            ekf = run_ekf_tracker(meas, init, cfg, sensors)
            metrics = compute_metrics(true_positions, ekf["position"])
            rmse[s_idx, run] = metrics["rmse_position_m"]

    return {
        "names": names,
        "mean_rmse": rmse.mean(axis=1),
        "std_rmse": rmse.std(axis=1, ddof=1),
        "mean_condition": mean_condition,
    }


def save_outputs(
    cfg: Config,
    t: np.ndarray,
    true_positions: np.ndarray,
    measurements: dict[str, np.ndarray],
    ekf: dict[str, np.ndarray],
    metrics: dict[str, float],
    geometry: dict[str, np.ndarray],
) -> None:
    cfg.output_folder.mkdir(exist_ok=True)

    tracking_path = cfg.output_folder / "tracking_results.csv"
    errors = np.linalg.norm(ekf["position"] - true_positions, axis=1)
    with tracking_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(
            [
                "time_s",
                "true_x_m",
                "true_y_m",
                "estimated_x_m",
                "estimated_y_m",
                "position_error_m",
                "nlos_anchor_count",
            ]
        )
        for k in range(len(t)):
            writer.writerow(
                [
                    f"{t[k]:.6f}",
                    f"{true_positions[k, 0]:.6f}",
                    f"{true_positions[k, 1]:.6f}",
                    f"{ekf['position'][k, 0]:.6f}",
                    f"{ekf['position'][k, 1]:.6f}",
                    f"{errors[k]:.6f}",
                    int(measurements["is_nlos"][k].sum()),
                ]
            )

    with (cfg.output_folder / "main_metrics.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(metrics.keys())
        writer.writerow([f"{value:.6f}" for value in metrics.values()])

    with (cfg.output_folder / "geometry_analysis.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(["geometry", "mean_rmse_m", "std_rmse_m", "mean_condition_number"])
        for i, name in enumerate(geometry["names"]):
            writer.writerow(
                [
                    name,
                    f"{geometry['mean_rmse'][i]:.6f}",
                    f"{geometry['std_rmse'][i]:.6f}",
                    f"{geometry['mean_condition'][i]:.6f}",
                ]
            )


def plot_factory_tracking(
    cfg: Config,
    true_positions: np.ndarray,
    measurements: dict[str, np.ndarray],
    ekf: dict[str, np.ndarray],
) -> None:
    fig, ax = plt.subplots(figsize=(10, 6), dpi=150)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(0, cfg.factory_size[0])
    ax.set_ylim(0, cfg.factory_size[1])
    ax.grid(True, alpha=0.3)

    for rect in OBSTACLES:
        ax.add_patch(
            Rectangle(
                (rect[0], rect[1]),
                rect[2],
                rect[3],
                facecolor="#b8b8b8",
                edgecolor="#3f3f3f",
                linewidth=1.2,
                label="Engel" if "Engel" not in ax.get_legend_handles_labels()[1] else None,
            )
        )

    ax.scatter(SENSORS_GOOD[:, 0], SENSORS_GOOD[:, 1], s=55, color="#0b4da2", label="UWB anchor")
    for idx, sensor in enumerate(SENSORS_GOOD, start=1):
        ax.text(sensor[0] + 0.35, sensor[1] + 0.35, f"A{idx}", color="#062f66", fontsize=8)

    nlos_count = measurements["is_nlos"].sum(axis=1)
    ax.scatter(
        true_positions[nlos_count > 0, 0],
        true_positions[nlos_count > 0, 1],
        s=12,
        color="#e36f10",
        alpha=0.6,
        label="NLOS olcum ani",
    )
    ax.plot(true_positions[:, 0], true_positions[:, 1], "k-", linewidth=2.0, label="Gercek AGV yolu")
    ax.plot(ekf["position"][:, 0], ekf["position"][:, 1], color="#c8102e", linewidth=1.7, label="EKF tahmini")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("2B fabrika zemini: TDOA tabanli AGV takibi")
    ax.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()
    fig.savefig(cfg.output_folder / "factory_tracking_map.png")
    plt.close(fig)


def plot_time_series(cfg: Config, t: np.ndarray, true_positions: np.ndarray, ekf: dict[str, np.ndarray]) -> None:
    error = np.linalg.norm(ekf["position"] - true_positions, axis=1)
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), dpi=150, sharex=True)

    axes[0].plot(t, true_positions[:, 0], "k-", linewidth=1.6, label="Gercek")
    axes[0].plot(t, ekf["position"][:, 0], "r--", linewidth=1.4, label="Tahmin")
    axes[0].set_ylabel("x [m]")
    axes[0].legend(loc="best")

    axes[1].plot(t, true_positions[:, 1], "k-", linewidth=1.6)
    axes[1].plot(t, ekf["position"][:, 1], "r--", linewidth=1.4)
    axes[1].set_ylabel("y [m]")

    axes[2].plot(t, error, color="#1b7f64", linewidth=1.4)
    axes[2].set_ylabel("konum hatasi [m]")
    axes[2].set_xlabel("zaman [s]")
    axes[2].set_title(f"RMSE = {np.sqrt(np.mean(error**2)):.2f} m")

    for ax in axes:
        ax.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(cfg.output_folder / "tracking_time_series.png")
    plt.close(fig)


def plot_geometry_analysis(cfg: Config, geometry: dict[str, np.ndarray]) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(10, 4), dpi=150)
    x = np.arange(len(geometry["names"]))

    axes[0].bar(x, geometry["mean_rmse"], yerr=geometry["std_rmse"], color="#2878b5", capsize=4)
    axes[0].set_xticks(x, geometry["names"])
    axes[0].set_ylabel("Ortalama RMSE [m]")
    axes[0].set_title("Sensor geometrisi etkisi")
    axes[0].grid(True, axis="y", alpha=0.3)

    axes[1].semilogy(x, geometry["mean_condition"], "o-", color="#b43b3b", linewidth=1.6)
    axes[1].set_xticks(x, geometry["names"])
    axes[1].set_ylabel("Ortalama cond(H^T H)")
    axes[1].set_title("TDOA geometri kosullanmasi")
    axes[1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(cfg.output_folder / "sensor_geometry_analysis.png")
    plt.close(fig)


def main() -> None:
    cfg = Config()
    cfg.output_folder.mkdir(exist_ok=True)
    rng = np.random.default_rng(407)

    t, truth_state = simulate_agv_trajectory(cfg)
    true_positions = truth_state[:, :2]

    measurements = generate_tdoa_measurements(true_positions, cfg, SENSORS_GOOD, rng)
    init = initialize_with_lse(measurements, cfg, SENSORS_GOOD)
    ekf = run_ekf_tracker(measurements, init, cfg, SENSORS_GOOD)

    metrics = compute_metrics(true_positions, ekf["position"])
    geometry = analyze_sensor_geometry(cfg, true_positions, rng)

    save_outputs(cfg, t, true_positions, measurements, ekf, metrics, geometry)
    plot_factory_tracking(cfg, true_positions, measurements, ekf)
    plot_time_series(cfg, t, true_positions, ekf)
    plot_geometry_analysis(cfg, geometry)

    print("AGV TDOA-EKF Python simulasyonu tamamlandi.")
    print(f"Ana senaryo konum RMSE: {metrics['rmse_position_m']:.3f} m")
    print(f"Ortalama NLOS anchor sayisi: {measurements['is_nlos'].sum(axis=1).mean():.2f} / {SENSORS_GOOD.shape[0]}")
    print(f"Ciktilar: {cfg.output_folder.resolve()}")


if __name__ == "__main__":
    main()

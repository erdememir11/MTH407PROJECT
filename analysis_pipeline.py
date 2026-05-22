"""
analysis_pipeline.py

Bu dosya baseline ve realistic gürültü senaryolarında ortak kullanılan analiz
akışını içerir. Doğrudan çalıştırılması gerekmez; `run_02_baseline_constant_noise.py`
ve `run_03_realistic_los_nlos_noise.py` bu dosyadaki `run_noise_analysis`
fonksiyonunu çağırır.

Ana görevleri:
1. 4 sensörlü 6 farklı geometriyi tek tek çalıştırmak.
2. Her geometri için gerçek yol vs EKF tahmini grafiği üretmek.
3. 4 sensörlü geometrilerin hata ve RMSE karşılaştırmalarını çizmek.
4. Sensör sayısı 4'ten 10'a çıktığında performans değişimini analiz etmek.
5. Grafiklerin yanında CSV özet tabloları üretmek.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from project_model import (
    Config,
    GEOMETRY_LABELS,
    SENSOR_GEOMETRIES_4,
    SENSOR_POOL_10,
    evaluate_layout,
    monte_carlo_layout,
    plot_case_map_and_error,
    simulate_robot_motion,
    write_rows,
)


# =============================================================================
# 1. ANA ANALİZ AKIŞI
# -----------------------------------------------------------------------------
# run_noise_analysis fonksiyonu bir gürültü senaryosunu baştan sona çalıştırır.
# Gürültü senaryosu parametre olarak verilir:
# - baseline_constant_los
# - realistic_los_nlos
# =============================================================================

def run_noise_analysis(noise_mode: str, output_dir: Path, title: str, seed_base: int) -> None:
    """Verilen gürültü modu için tüm 4 sensör ve 4-10 sensör analiz çıktılarını üretir."""
    cfg = Config()
    output_dir.mkdir(parents=True, exist_ok=True)
    t, truth_state = simulate_robot_motion(cfg)
    true_positions = truth_state[:, :2]

    geometry_rows = []
    error_series = {}

    for idx, (name, sensors) in enumerate(SENSOR_GEOMETRIES_4.items(), start=1):
        result = evaluate_layout(cfg, true_positions, sensors, np.random.default_rng(seed_base + idx), noise_mode)
        mc = monte_carlo_layout(cfg, true_positions, sensors, noise_mode, base_seed=seed_base + 100 * idx)
        error = plot_case_map_and_error(
            cfg,
            t,
            true_positions,
            sensors,
            result,
            output_dir / f"{idx:02d}_{name}_true_vs_estimate.png",
            f"{title} - {GEOMETRY_LABELS[name]}",
        )
        error_series[name] = error
        geometry_rows.append(
            {
                "geometry": name,
                "label": GEOMETRY_LABELS[name],
                "sensor_count": sensors.shape[0],
                "single_run_rmse_m": result["metrics"]["rmse_position_m"],
                "single_run_mean_error_m": result["metrics"]["mean_position_error_m"],
                "single_run_max_error_m": result["metrics"]["max_position_error_m"],
                "single_run_mean_nlos_anchor_count": result["metrics"]["mean_nlos_anchor_count"],
                **mc,
            }
        )

    write_rows(output_dir / "four_sensor_geometry_summary.csv", geometry_rows)
    plot_four_sensor_error_comparison(t, error_series, output_dir / "four_sensor_error_time_comparison.png", title)
    plot_four_sensor_rmse_comparison(geometry_rows, output_dir / "four_sensor_rmse_comparison.png", title)

    count_rows = []
    for count in range(4, SENSOR_POOL_10.shape[0] + 1):
        sensors = SENSOR_POOL_10[:count]
        result = evaluate_layout(cfg, true_positions, sensors, np.random.default_rng(seed_base + 1000 + count), noise_mode)
        mc = monte_carlo_layout(cfg, true_positions, sensors, noise_mode, base_seed=seed_base + 2000 + 100 * count)
        count_rows.append(
            {
                "sensor_count": count,
                "single_run_rmse_m": result["metrics"]["rmse_position_m"],
                "single_run_mean_error_m": result["metrics"]["mean_position_error_m"],
                "single_run_max_error_m": result["metrics"]["max_position_error_m"],
                "single_run_mean_nlos_anchor_count": result["metrics"]["mean_nlos_anchor_count"],
                **mc,
            }
        )

    write_rows(output_dir / "sensor_count_4_to_10_summary.csv", count_rows)
    plot_sensor_count_comparison(count_rows, output_dir / "sensor_count_4_to_10_comparison.png", title)
    print(f"{title} outputs generated: {output_dir.resolve()}")


# =============================================================================
# 2. 4 SENSÖRLÜ GEOMETRİLERİN HATA KARŞILAŞTIRMASI
# -----------------------------------------------------------------------------
# Bu grafik, G1-G6 geometrilerinin zamana bağlı konum hatalarını aynı eksende
# gösterir. Böylece sadece RMSE değil, hatanın rotanın hangi kısmında arttığı da
# görülebilir.
# =============================================================================

def plot_four_sensor_error_comparison(t: np.ndarray, error_series: dict[str, np.ndarray], output_path: Path, title: str) -> None:
    """4 sensörlü tüm geometrilerin zaman-hata eğrilerini tek grafikte karşılaştırır."""
    fig, ax = plt.subplots(figsize=(11, 6), dpi=150)
    for name, error in error_series.items():
        ax.plot(t, error, linewidth=1.4, label=GEOMETRY_LABELS[name])
    ax.axvspan(10.0, 15.0, color="#f59e0b", alpha=0.14, label="Pickup wait")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position error [m]")
    ax.set_title(f"{title}: 4-Sensor Error Comparison")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


# =============================================================================
# 3. 4 SENSÖRLÜ GEOMETRİLERİN RMSE KARŞILAŞTIRMASI
# -----------------------------------------------------------------------------
# Bu bölüm, her geometri için Monte Carlo ortalama RMSE ve standart sapmayı
# gösteren özet grafiği üretir. Tekil koşu RMSE'si de çizgi olarak eklenir.
# =============================================================================

def plot_four_sensor_rmse_comparison(rows: list[dict[str, object]], output_path: Path, title: str) -> None:
    """4 sensör geometrilerinin Monte Carlo ortalama RMSE değerlerini çizer."""
    labels = [str(row["label"]).replace(" ", "\n") for row in rows]
    mean_rmse = np.array([float(row["mean_rmse_m"]) for row in rows])
    std_rmse = np.array([float(row["std_rmse_m"]) for row in rows])
    single_rmse = np.array([float(row["single_run_rmse_m"]) for row in rows])
    x = np.arange(len(rows))

    fig, ax = plt.subplots(figsize=(12, 6), dpi=150)
    ax.bar(x, mean_rmse, yerr=std_rmse, capsize=4, color="#2563eb", alpha=0.75, label="Monte Carlo mean RMSE")
    ax.plot(x, single_rmse, "o-", color="#dc2626", linewidth=1.4, label="Single-run RMSE")
    ax.set_xticks(x, labels)
    ax.set_ylabel("RMSE [m]")
    ax.set_title(f"{title}: 4-Sensor Geometry RMSE")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


# =============================================================================
# 4. SENSÖR SAYISI ANALİZİ
# -----------------------------------------------------------------------------
# Bu bölüm, sensör sayısı 4'ten 10'a artırıldığında RMSE, geometri koşul sayısı
# ve tekil koşudaki ortalama NLOS sensör sayısının nasıl değiştiğini gösterir.
# =============================================================================

def plot_sensor_count_comparison(rows: list[dict[str, object]], output_path: Path, title: str) -> None:
    """4-10 sensör sayısı analizini üç alt grafik halinde üretir."""
    counts = np.array([int(row["sensor_count"]) for row in rows])
    mean_rmse = np.array([float(row["mean_rmse_m"]) for row in rows])
    std_rmse = np.array([float(row["std_rmse_m"]) for row in rows])
    condition = np.array([float(row["mean_condition_number"]) for row in rows])
    nlos = np.array([float(row["single_run_mean_nlos_anchor_count"]) for row in rows])

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), dpi=150, sharex=True)
    axes[0].errorbar(counts, mean_rmse, yerr=std_rmse, marker="o", linewidth=1.6, capsize=4, color="#2563eb")
    axes[0].set_ylabel("Mean RMSE [m]")
    axes[0].set_title(f"{title}: Sensor Count Comparison")
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

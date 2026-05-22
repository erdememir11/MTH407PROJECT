"""
generate_01_environment.py

Bu dosya yalnızca simülasyon ortamını ve beklenen robot hareketini görselleştirir.
Sensör yerleşimi, TDOA ölçümü, LSE veya EKF hesabı yapmaz.

Üretilen çıktı klasörü:
    outputs/01_environment/

Üretilen dosyalar:
1. factory_environment_details.png
   Fabrika bölgelerini, dış sınırı, iç duvarları, kapıları ve şişirilmiş
   engelleri gösterir.

2. expected_robot_motion.png
   Robotun Parking Docks -> Pickup -> Gate-2 -> Drop-off görev hareketini
   fabrika ortamı üzerinde gösterir.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt

from project_model import Config, MISSION_POINTS, draw_environment_base, mark_mission_points, simulate_robot_motion


# =============================================================================
# 1. ÇALIŞTIRMA AKIŞI
# -----------------------------------------------------------------------------
# main fonksiyonu ortam klasörünü oluşturur ve iki temel ortam görselini üretir.
# =============================================================================

def main() -> None:
    """Ortam ve beklenen robot hareketi çıktılarını üretir."""
    cfg = Config()
    output_dir = cfg.output_folder / "01_environment"
    output_dir.mkdir(parents=True, exist_ok=True)

    plot_factory_environment(cfg, output_dir / "factory_environment_details.png")
    plot_expected_robot_motion(cfg, output_dir / "expected_robot_motion.png")
    print(f"Environment outputs generated: {output_dir.resolve()}")


# =============================================================================
# 2. FABRİKA ORTAMI GÖRSELİ
# -----------------------------------------------------------------------------
# Bu görsel robot rotasını veya takip tahminini içermez. Amaç, raporda
# kullanılacak yalın fabrika geometrisi görselini üretmektir.
# =============================================================================

def plot_factory_environment(cfg: Config, output_path: Path) -> None:
    """Fabrikanın bölge, duvar, kapı ve engel yapısını çizer."""
    fig, ax = plt.subplots(figsize=(12, 7), dpi=150)
    draw_environment_base(ax, cfg)
    ax.set_title("Factory Environment: Boundaries, Regions, Doors and Inflated Obstacles")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


# =============================================================================
# 3. BEKLENEN ROBOT HAREKETİ GÖRSELİ
# -----------------------------------------------------------------------------
# Bu görsel gerçek kabul edilen görev rotasını gösterir. EKF tahmini veya
# ölçüm gürültüsü yoktur; yalnızca ideal robot hareketi çizilir.
# =============================================================================

def plot_expected_robot_motion(cfg: Config, output_path: Path) -> None:
    """Robotun görev hareketini fabrika haritası üzerinde gösterir."""
    t, state = simulate_robot_motion(cfg)
    positions = state[:, :2]

    fig, ax = plt.subplots(figsize=(12, 7), dpi=150)
    draw_environment_base(ax, cfg)
    ax.plot(MISSION_POINTS[:, 0], MISSION_POINTS[:, 1], "o--", color="#92400e", linewidth=1.4, label="Mission points")
    ax.plot(positions[:, 0], positions[:, 1], color="black", linewidth=2.0, label="Expected robot motion")
    mark_mission_points(ax)
    ax.set_title(f"Expected Robot Motion, total time={t[-1]:.1f} s")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)


if __name__ == "__main__":
    main()

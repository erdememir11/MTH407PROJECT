"""
Fabrika geometrisi gorsellestirme kodu

Bu dosya yalnizca fabrika ortamini anlamak icin hazirlanmistir; robot takibi,
TDOA olcumu, EKF veya sensor performans analizi icermez.

Ortam tanimi:

1. Dis sinir
   - Fabrika 50 m x 30 m boyutunda dikdortgen bir alandir.
   - Koordinat sistemi metre cinsindedir.
   - Orijin sol-alt kosededir: (0, 0).
   - Sag-ust kose: (50, 30).
   - Robot bu sinirlarin disina cikamaz.

2. Bolgeler
   - Parking Docks:
     0 <= x <= 25, 20 <= y <= 30.
     Robotun goreve basladigi park/dock bolgesidir.
   - Battery Loading / Pickup:
     0 <= x <= 25, 10 <= y <= 20.
     Robotun bataryayi teslim aldigi pickup bolgesidir.
   - Main transit aisle:
     25 <= x <= 50, 10 <= y <= 30.
     Robotun ana gecis koridorudur.
   - Lower service area:
     0 <= x <= 50, 0 <= y <= 10.
     Ana gorev rotasinin dogrudan tercih etmedigi alt servis alanidir.
   - Start point:
     (5, 25). Robotun Parking Docks icindeki baslangic noktasidir.
   - Pickup point:
     (5, 15). Robotun bataryayi aldigi noktadir.
   - Drop-off point:
     (45, 10). Batarya yerlestirme/hedef noktasidir.

3. Ic duvarlar / bolmeler
   - B1: y = 20, 0 <= x <= 25.
     Parking Docks ile Battery Loading / Pickup arasindaki yatay bolmedir.
     Gate-1 nedeniyle x = 4..6 arasi aciktir.
   - B2: x = 25, 10 <= y <= 30.
     Sol bolgeler ile main transit aisle arasindaki dikey bolmedir.
     Gate-2 nedeniyle y = 16.6..19.0 arasi aciktir.
   - B3: y = 10, 0 <= x <= 40.
     Alt servis bolgesi ile ust calisma bolgelerini ayiran yatay bolmedir.
     x > 40 tarafi drop-off yaklasimi icin acik birakilmistir.
   - B4: x = 40, 0 <= y <= 10.
     Drop-off tarafindaki dikey alt bolmedir.

4. Kapilar / acikliklar
   - Gate-1:
     B1 uzerinde x = 4..6 araliginda aciklik.
     Robotun Parking Docks bolgesinden Battery Loading / Pickup tarafina gecmesini saglar.
   - Gate-2:
     B2 uzerinde y = 16.6..19.0 araliginda aciklik.
     Robotun sol bolgeden main transit aisle bolgesine gecmesini saglar.
   - Drop-off approach:
     B3 bolmesi x <= 40 ile sinirlandirilmistir.
     Boylece hedefe x > 40 tarafindan yaklasmak mumkundur.

5. Engeller / yasak bolgeler
   - Bu projede ic duvar ve bolmeler robot icin engel kabul edilir.
   - Robot noktasal modellenir; ancak guvenli gecis icin duvarlar sisirilir.
   - Robot yaricapi: 0.35 m.
   - Guvenlik mesafesi: 0.25 m.
   - Toplam guvenlik payi: 0.60 m.
   - Ham duvar kalinligi: 0.20 m.
   - Gorselde kirmizi seffaf bantlar sisirilmis yasak bolgeleri gosterir.

Uretilen cikti:
    outputs/factory_geometry_only.png

Calistirma:
    python factory_geometry_visual.py
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


@dataclass(frozen=True)
class FactoryGeometry:
    width: float = 50.0
    height: float = 30.0
    robot_radius: float = 0.35
    safety_margin: float = 0.25
    wall_thickness: float = 0.20
    output_folder: Path = Path("outputs")

    @property
    def safe_distance(self) -> float:
        return self.robot_radius + self.safety_margin

    @property
    def inflated_half_width(self) -> float:
        return self.wall_thickness / 2.0 + self.safe_distance


def inflated_obstacles(cfg: FactoryGeometry) -> list[tuple[str, tuple[float, float, float, float]]]:
    half = cfg.inflated_half_width
    return [
        ("B1-left", (0.0, 20.0 - half, 4.0, 20.0 + half)),
        ("B1-right", (6.0, 20.0 - half, 25.0, 20.0 + half)),
        ("B2-lower", (25.0 - half, 10.0, 25.0 + half, 16.6)),
        ("B2-upper", (25.0 - half, 19.0, 25.0 + half, 30.0)),
        ("B3", (0.0, 10.0 - half, 40.0, 10.0 + half)),
        ("B4", (40.0 - half, 0.0, 40.0 + half, 10.0)),
    ]


def draw_factory_geometry(cfg: FactoryGeometry) -> Path:
    cfg.output_folder.mkdir(exist_ok=True)
    output_path = cfg.output_folder / "factory_geometry_only.png"

    fig, ax = plt.subplots(figsize=(13, 8), dpi=160)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(-1, cfg.width + 1)
    ax.set_ylim(-1, cfg.height + 1)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Fabrika geometrisi: dis sinir, ic duvarlar, engeller, kapilar ve bolgeler")
    ax.grid(True, color="#d4d4d4", linewidth=0.6, alpha=0.7)

    draw_zones(ax)
    draw_outer_boundary(ax, cfg)
    draw_inflated_obstacles(ax, cfg)
    draw_inner_walls(ax)
    draw_gates(ax)
    draw_key_points(ax)
    draw_legend(ax)

    fig.tight_layout()
    fig.savefig(output_path)
    plt.close(fig)
    return output_path


def draw_zones(ax) -> None:
    zones = [
        ((0.0, 20.0), 25.0, 10.0, "#dbeafe", "Parking Docks"),
        ((0.0, 10.0), 25.0, 10.0, "#fde68a", "Battery Loading / Pickup"),
        ((25.0, 10.0), 25.0, 20.0, "#dcfce7", "Main transit aisle"),
        ((0.0, 0.0), 50.0, 10.0, "#f3f4f6", "Lower service area"),
    ]

    for xy, width, height, color, label in zones:
        ax.add_patch(
            Rectangle(
                xy,
                width,
                height,
                facecolor=color,
                edgecolor="#ffffff",
                linewidth=1.0,
                alpha=0.65,
                zorder=1,
            )
        )
        ax.text(
            xy[0] + width / 2,
            xy[1] + height / 2,
            label,
            ha="center",
            va="center",
            fontsize=9,
            color="#111827",
            zorder=2,
        )


def draw_outer_boundary(ax, cfg: FactoryGeometry) -> None:
    ax.add_patch(
        Rectangle(
            (0.0, 0.0),
            cfg.width,
            cfg.height,
            fill=False,
            edgecolor="#111827",
            linewidth=2.5,
            zorder=8,
        )
    )
    ax.text(0.5, cfg.height + 0.35, "Dis sinir: 50 m x 30 m", fontsize=9, color="#111827")


def draw_inflated_obstacles(ax, cfg: FactoryGeometry) -> None:
    for label, (xmin, ymin, xmax, ymax) in inflated_obstacles(cfg):
        ax.add_patch(
            Rectangle(
                (xmin, ymin),
                xmax - xmin,
                ymax - ymin,
                facecolor="#ef4444",
                edgecolor="#991b1b",
                linewidth=0.8,
                alpha=0.28,
                zorder=4,
            )
        )
        ax.text(
            (xmin + xmax) / 2,
            (ymin + ymax) / 2,
            label,
            ha="center",
            va="center",
            fontsize=7,
            color="#7f1d1d",
            zorder=5,
        )


def draw_inner_walls(ax) -> None:
    wall_color = "#111827"
    wall_width = 3.0

    # B1 with Gate-1 gap x=4..6.
    ax.plot([0.0, 4.0], [20.0, 20.0], color=wall_color, linewidth=wall_width, zorder=6)
    ax.plot([6.0, 25.0], [20.0, 20.0], color=wall_color, linewidth=wall_width, zorder=6)

    # B2 with Gate-2 gap y=16.6..19.0.
    ax.plot([25.0, 25.0], [10.0, 16.6], color=wall_color, linewidth=wall_width, zorder=6)
    ax.plot([25.0, 25.0], [19.0, 30.0], color=wall_color, linewidth=wall_width, zorder=6)

    # B3 and B4.
    ax.plot([0.0, 40.0], [10.0, 10.0], color=wall_color, linewidth=wall_width, zorder=6)
    ax.plot([40.0, 40.0], [0.0, 10.0], color=wall_color, linewidth=wall_width, zorder=6)

    wall_labels = [
        ("B1", 15.5, 20.8),
        ("B2", 25.8, 24.0),
        ("B3", 20.0, 10.8),
        ("B4", 40.8, 5.0),
    ]
    for text, x, y in wall_labels:
        ax.text(x, y, text, fontsize=8, fontweight="bold", color=wall_color, zorder=7)


def draw_gates(ax) -> None:
    gate_color = "#16a34a"
    gate_style = dict(color=gate_color, linewidth=5.0, solid_capstyle="round", zorder=9)

    ax.plot([4.0, 6.0], [20.0, 20.0], **gate_style)
    ax.text(5.0, 20.95, "Gate-1", ha="center", fontsize=8, color="#166534", fontweight="bold")

    ax.plot([25.0, 25.0], [16.6, 19.0], **gate_style)
    ax.text(26.1, 17.8, "Gate-2", va="center", fontsize=8, color="#166534", fontweight="bold")

    ax.plot([40.0, 50.0], [10.0, 10.0], color="#22c55e", linewidth=3.0, linestyle="--", zorder=9)
    ax.text(44.7, 10.7, "Drop-off approach", ha="center", fontsize=8, color="#166534", fontweight="bold")


def draw_key_points(ax) -> None:
    start = (5.0, 25.0)
    pickup = (5.0, 15.0)
    goal = (45.0, 10.0)
    ax.scatter([start[0]], [start[1]], s=90, color="#2563eb", zorder=10)
    ax.text(start[0] + 0.5, start[1] + 0.5, "Start / Parking Docks\n(5, 25)", fontsize=8, color="#1e40af")

    ax.scatter([pickup[0]], [pickup[1]], s=90, marker="D", color="#f59e0b", zorder=10)
    ax.text(pickup[0] + 0.5, pickup[1] + 0.5, "Pickup / Battery Loading\n(5, 15)", fontsize=8, color="#92400e")

    ax.scatter([goal[0]], [goal[1]], s=150, marker="*", color="#dc2626", zorder=10)
    ax.text(goal[0] + 0.7, goal[1] - 1.2, "Drop-off\n(45, 10)", fontsize=8, color="#991b1b")


def draw_legend(ax) -> None:
    handles = [
        Rectangle((0, 0), 1, 1, facecolor="#dbeafe", edgecolor="none", alpha=0.65, label="Bolge"),
        Line2D([0], [0], color="#111827", linewidth=3.0, label="Ic duvar / bolme"),
        Rectangle((0, 0), 1, 1, facecolor="#ef4444", edgecolor="#991b1b", alpha=0.28, label="Sisirilmis engel"),
        Line2D([0], [0], color="#16a34a", linewidth=5.0, label="Kapi / aciklik"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#2563eb", markersize=8, label="Baslangic / Parking"),
        Line2D([0], [0], marker="D", color="w", markerfacecolor="#f59e0b", markersize=8, label="Pickup"),
        Line2D([0], [0], marker="*", color="w", markerfacecolor="#dc2626", markersize=12, label="Drop-off"),
    ]
    ax.legend(handles=handles, loc="center left", bbox_to_anchor=(1.02, 0.5), frameon=True)


def main() -> None:
    cfg = FactoryGeometry()
    output_path = draw_factory_geometry(cfg)
    print(f"Fabrika geometrisi gorseli olusturuldu: {output_path.resolve()}")


if __name__ == "__main__":
    main()

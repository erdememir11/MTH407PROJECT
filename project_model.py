"""
project_model.py

Bu dosya projenin ortak model ve algoritma kütüphanesidir.
Tek başına bir çıktı üretmez; diğer çalıştırma dosyaları tarafından içe aktarılır.

İçerdiği ana görevler:
1. Fabrika ortamı, robot görevi ve sensör geometrilerini tanımlar.
2. Robotun beklenen hareketini simülasyon olarak üretir.
3. UWB/TDOA ölçümlerini LOS veya LOS/NLOS gürültü senaryosuna göre üretir.
4. İlk konumu LSE yöntemiyle tahmin eder.
5. EKF ile robotun konumunu ve hızını zaman içinde takip eder.
6. RMSE, koşul sayısı ve NLOS gibi performans metriklerini hesaplar.
7. Ortak grafik çizim fonksiyonlarını sağlar.

Bu dosyanın ayrı tutulmasının nedeni, baseline ve realistic gürültü
senaryolarında aynı fiziksel modelin ve aynı takip algoritmasının
tekrar kullanılmasını sağlamaktır.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


# =============================================================================
# 1. GENEL KONFİGÜRASYON
# -----------------------------------------------------------------------------
# Config sınıfı, tüm simülasyon boyunca kullanılan ortak parametreleri tutar.
# Buradaki değerler değiştirildiğinde robot hızı, fabrika boyutu, gürültü
# seviyesi, EKF süreç gürültüsü ve Monte Carlo tekrar sayısı gibi temel
# deney ayarları değişmiş olur.
# =============================================================================

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
    monte_carlo_runs: int = 30

    @property
    def d_safe(self) -> float:
        return self.robot_radius + self.safety_margin

    @property
    def sigma_toa_los(self) -> float:
        return 0.25 / self.c

    @property
    def sigma_toa_nlos(self) -> float:
        return 1.50 / self.c


# =============================================================================
# 2. GÖREV NOKTALARI VE ROBOT ROTASI
# -----------------------------------------------------------------------------
# MISSION_POINTS robotun fiziksel görev yolunu tanımlar:
# Parking Docks -> Pickup -> Gate-2 -> Drop-off.
# Simülasyon bu noktaları sırayla takip eder. Pickup noktasında ayrıca
# Config.pickup_wait_time kadar bekleme yapılır.
# =============================================================================

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


# =============================================================================
# 3. SENSÖR GEOMETRİLERİ
# -----------------------------------------------------------------------------
# SENSOR_GEOMETRIES_4, Word şablonunda verilen 6 farklı 4 sensörlü geometriyi
# içerir. Bu geometriler projenin ana karşılaştırma konusudur.
#
# SENSOR_POOL_10 ise sensör sayısı 4'ten 10'a çıkarıldığında kullanılacak
# maksimum sensör havuzudur. Analizde ilk 4, ilk 5, ..., ilk 10 sensör sırasıyla
# denenir.
# =============================================================================

SENSOR_GEOMETRIES_4 = {
    "G1_balanced_corner_coverage": np.array(
        [[1.0, 29.0], [49.0, 29.0], [49.0, 1.0], [1.0, 1.0]],
        dtype=float,
    ),
    "G2_task_oriented_route_coverage": np.array(
        [[1.0, 29.0], [8.0, 16.0], [27.0, 18.0], [46.0, 12.0]],
        dtype=float,
    ),
    "G3_poor_bottom_cluster": np.array(
        [[1.0, 29.0], [16.0, 2.0], [32.0, 2.0], [48.0, 2.0]],
        dtype=float,
    ),
    "G4_b3_wall_aware_coverage": np.array(
        [[1.0, 29.0], [49.0, 29.0], [1.0, 11.2], [39.0, 11.2]],
        dtype=float,
    ),
    "G5_left_bottom_b3_right": np.array(
        [[1.0, 29.0], [49.0, 29.0], [1.0, 1.0], [39.0, 11.2]],
        dtype=float,
    ),
    "G6_right_bottom_b3_left": np.array(
        [[1.0, 29.0], [49.0, 29.0], [49.0, 1.0], [1.0, 11.2]],
        dtype=float,
    ),
}


SENSOR_POOL_10 = np.array(
    [
        [1.0, 29.0],
        [49.0, 29.0],
        [49.0, 1.0],
        [1.0, 1.0],
        [8.0, 16.0],
        [27.0, 18.0],
        [46.0, 12.0],
        [1.0, 11.2],
        [39.0, 11.2],
        [25.0, 18.0],
    ],
    dtype=float,
)


GEOMETRY_LABELS = {
    "G1_balanced_corner_coverage": "G1 Balanced Corner Coverage",
    "G2_task_oriented_route_coverage": "G2 Task-Oriented Route Coverage",
    "G3_poor_bottom_cluster": "G3 Poor Bottom Cluster",
    "G4_b3_wall_aware_coverage": "G4 B3 Wall-Aware Coverage",
    "G5_left_bottom_b3_right": "G5 Left-Bottom + B3-Right",
    "G6_right_bottom_b3_left": "G6 Right-Bottom + B3-Left",
}


# =============================================================================
# 4. FABRİKA ENGEL MODELİ
# -----------------------------------------------------------------------------
# Fabrika duvarları ve bölmeleri robot için yasak bölgedir. Robot noktasal
# modellendiği için duvarlar robot yarıçapı + güvenlik payı kadar şişirilir.
# inflated_obstacles fonksiyonu bu şişirilmiş yasak dikdörtgenleri döndürür.
# =============================================================================

def inflated_obstacles(cfg: Config) -> list[tuple[float, float, float, float]]:
    """Şişirilmiş duvar/engel dikdörtgenlerini (xmin, ymin, xmax, ymax) döndürür."""
    half = cfg.wall_thickness / 2.0 + cfg.d_safe
    return [
        (0.0, 20.0 - half, 4.0, 20.0 + half),
        (6.0, 20.0 - half, 25.0, 20.0 + half),
        (25.0 - half, 10.0, 25.0 + half, 16.6),
        (25.0 - half, 19.0, 25.0 + half, 30.0),
        (0.0, 10.0 - half, 40.0, 10.0 + half),
        (40.0 - half, 0.0, 40.0 + half, 10.0),
    ]


# =============================================================================
# 5. ROBOT HAREKET SİMÜLASYONU
# -----------------------------------------------------------------------------
# Bu bölüm robotun gerçek kabul edilen yolunu üretir. Takip algoritması bu
# gerçek yolu doğrudan bilmez; gerçek yol yalnızca ölçüm üretmek ve sonradan
# hata hesabı yapmak için kullanılır.
# =============================================================================

def simulate_robot_motion(cfg: Config) -> tuple[np.ndarray, np.ndarray]:
    """Robotun görev fazlarına göre gerçek konum ve hız dizisini üretir."""
    obstacles = inflated_obstacles(cfg)
    samples: list[np.ndarray] = []
    current = MISSION_POINTS[PARKING_START_INDEX].copy()
    samples.append(np.array([current[0], current[1], 0.0, 0.0], dtype=float))

    current = append_motion_segment(samples, current, MISSION_POINTS[PICKUP_INDEX], cfg, obstacles)

    wait_steps = int(round(cfg.pickup_wait_time / cfg.dt))
    for _ in range(wait_steps):
        samples.append(np.array([current[0], current[1], 0.0, 0.0], dtype=float))

    current = append_motion_segment(samples, current, MISSION_POINTS[GATE2_INDEX], cfg, obstacles)
    append_motion_segment(samples, current, MISSION_POINTS[DROPOFF_INDEX], cfg, obstacles)

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
    """İki görev noktası arasında sabit hızlı, çarpışmasız hareket örnekleri ekler."""
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
            raise RuntimeError(f"Collision on segment: start={start}, target={target}, candidate={next_position}")
        samples.append(np.array([next_position[0], next_position[1], velocity[0], velocity[1]], dtype=float))
        current = next_position


def inside_map(p: np.ndarray, cfg: Config) -> bool:
    """Bir noktanın fabrika sınırları içinde kalıp kalmadığını kontrol eder."""
    return 0.0 <= p[0] <= cfg.factory_size[0] and 0.0 <= p[1] <= cfg.factory_size[1]


def collision_point(p: np.ndarray, obstacles: list[tuple[float, float, float, float]]) -> bool:
    """Bir noktanın şişirilmiş engellerden birinin içine düşüp düşmediğini kontrol eder."""
    return any(xmin <= p[0] <= xmax and ymin <= p[1] <= ymax for xmin, ymin, xmax, ymax in obstacles)


# =============================================================================
# 6. UWB/TDOA ÖLÇÜM ÜRETİMİ
# -----------------------------------------------------------------------------
# Bu bölüm sensör konumlarına ve robotun gerçek konumuna göre TDOA menzil farkı
# ölçümleri üretir. İki gürültü modu desteklenir:
# - baseline_constant_los: Tüm bağlantılar düşük gürültülü LOS kabul edilir.
# - realistic_los_nlos: Engel kesişimi varsa NLOS kabul edilir ve gürültü artar.
# =============================================================================

def generate_tdoa_measurements(
    positions: np.ndarray,
    cfg: Config,
    sensors: np.ndarray,
    rng: np.random.Generator,
    noise_mode: str,
) -> dict[str, np.ndarray]:
    """Verilen sensör geometrisi için gürültülü TDOA ölçüm dizisi üretir."""
    if noise_mode not in {"baseline_constant_los", "realistic_los_nlos"}:
        raise ValueError(f"Unknown noise mode: {noise_mode}")

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

        if noise_mode == "realistic_los_nlos":
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


# =============================================================================
# 7. LSE İLE BAŞLANGIÇ TAHMİNİ
# -----------------------------------------------------------------------------
# EKF'nin başlayabilmesi için ilk konum ve kovaryans tahminine ihtiyaç vardır.
# Bu bölüm ilk TDOA ölçümlerinden Gauss-Newton tabanlı LSE ile başlangıç konumu
# üretir, ardından ilk hız bileşenlerini ilk birkaç LSE konumundan çıkarır.
# =============================================================================

def initialize_with_lse(measurements: dict[str, np.ndarray], cfg: Config, sensors: np.ndarray) -> dict[str, np.ndarray]:
    """İlk TDOA ölçümlerinden EKF başlangıç durumu ve kovaryansını üretir."""
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


def estimate_position_lse(z: np.ndarray, r_mat: np.ndarray, cfg: Config, sensors: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Tek bir zaman adımı için Gauss-Newton LSE konum tahmini yapar."""
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


# =============================================================================
# 8. EKF TAKİP ALGORİTMASI
# -----------------------------------------------------------------------------
# TDOA ölçüm modeli doğrusal olmadığı için klasik Kalman filtresi yerine EKF
# kullanılır. Hareket modeli sabit hızlıdır; ölçüm modeli her adımda Jacobian
# ile doğrusallaştırılır.
# =============================================================================

def run_ekf_tracker(measurements: dict[str, np.ndarray], init: dict[str, np.ndarray], cfg: Config, sensors: np.ndarray) -> dict[str, np.ndarray]:
    """LSE başlangıcından sonra TDOA ölçümleriyle EKF konum/hız takibi yapar."""
    k_count = measurements["z_range_diff"].shape[0]
    state = np.zeros((k_count, 4), dtype=float)
    cov = np.zeros((k_count, 4, 4), dtype=float)
    innovation_norm = np.zeros(k_count, dtype=float)

    state[0] = init["x0"]
    cov[0] = init["p0"]
    dt = cfg.dt
    f = np.array([[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
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


def tdoa_measurement_model(position: np.ndarray, sensors: np.ndarray, reference_sensor: int) -> tuple[np.ndarray, np.ndarray]:
    """TDOA menzil farkı ölçüm fonksiyonunu ve konuma göre Jacobian'ını hesaplar."""
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


# =============================================================================
# 9. PERFORMANS METRİKLERİ VE DENEY KOŞULARI
# -----------------------------------------------------------------------------
# Bu bölüm tek bir sensör yerleşimini çalıştırmak, Monte Carlo tekrarı yapmak,
# RMSE ve sensör geometrisi koşul sayısı gibi metrikleri hesaplamak için
# kullanılır.
# =============================================================================

def compute_metrics(true_positions: np.ndarray, estimated_positions: np.ndarray) -> dict[str, float]:
    """Gerçek ve tahmin edilen konumlar arasındaki temel hata metriklerini hesaplar."""
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
    noise_mode: str,
) -> dict[str, object]:
    """Bir sensör geometrisi ve gürültü modu için tüm takip zincirini çalıştırır."""
    measurements = generate_tdoa_measurements(true_positions, cfg, sensors, rng, noise_mode)
    init = initialize_with_lse(measurements, cfg, sensors)
    ekf = run_ekf_tracker(measurements, init, cfg, sensors)
    metrics = compute_metrics(true_positions, ekf["position"])
    metrics["mean_nlos_anchor_count"] = float(measurements["is_nlos"].sum(axis=1).mean())
    metrics["mean_condition_number"] = mean_condition_number(true_positions, sensors, cfg)
    return {"measurements": measurements, "init": init, "ekf": ekf, "metrics": metrics}


def monte_carlo_layout(cfg: Config, true_positions: np.ndarray, sensors: np.ndarray, noise_mode: str, base_seed: int) -> dict[str, float]:
    """Aynı sensör geometrisini farklı rastgele gürültülerle tekrar ederek ortalama performansı hesaplar."""
    rmse = np.zeros(cfg.monte_carlo_runs, dtype=float)
    mean_error = np.zeros(cfg.monte_carlo_runs, dtype=float)
    max_error = np.zeros(cfg.monte_carlo_runs, dtype=float)
    for run in range(cfg.monte_carlo_runs):
        result = evaluate_layout(cfg, true_positions, sensors, np.random.default_rng(base_seed + run), noise_mode)
        rmse[run] = result["metrics"]["rmse_position_m"]
        mean_error[run] = result["metrics"]["mean_position_error_m"]
        max_error[run] = result["metrics"]["max_position_error_m"]
    return {
        "mean_rmse_m": float(rmse.mean()),
        "std_rmse_m": float(rmse.std(ddof=1)),
        "mean_error_m": float(mean_error.mean()),
        "max_error_m": float(max_error.mean()),
        "mean_condition_number": mean_condition_number(true_positions, sensors, cfg),
    }


def mean_condition_number(true_positions: np.ndarray, sensors: np.ndarray, cfg: Config) -> float:
    """TDOA Jacobian geometrisinin ortalama koşul sayısını hesaplar."""
    values = []
    for p in true_positions:
        _, h_pos = tdoa_measurement_model(p, sensors, cfg.reference_sensor)
        values.append(np.linalg.cond(h_pos.T @ h_pos + 1e-9 * np.eye(2)))
    return float(np.mean(values))


# =============================================================================
# 10. GEOMETRİK KESİŞİM YARDIMCILARI
# -----------------------------------------------------------------------------
# NLOS kararını vermek için robot-sensör doğru parçasının şişirilmiş engellerle
# kesişip kesişmediği kontrol edilir. Aşağıdaki küçük yardımcı fonksiyonlar bu
# geometrik testi yapar.
# =============================================================================

def line_intersects_any_obstacle(p1: np.ndarray, p2: np.ndarray, obstacles: list[tuple[float, float, float, float]]) -> bool:
    """Bir doğru parçasının herhangi bir şişirilmiş engelle kesişip kesişmediğini döndürür."""
    return any(segment_intersects_rect(p1, p2, rect) for rect in obstacles)


def segment_intersects_rect(p1: np.ndarray, p2: np.ndarray, rect: tuple[float, float, float, float]) -> bool:
    """Bir doğru parçasının bir dikdörtgenle kesişip kesişmediğini kontrol eder."""
    xmin, ymin, xmax, ymax = rect
    if xmin <= p1[0] <= xmax and ymin <= p1[1] <= ymax:
        return True
    if xmin <= p2[0] <= xmax and ymin <= p2[1] <= ymax:
        return True
    corners = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    return any(segments_intersect(p1, p2, corners[a], corners[b]) for a, b in edges)


def segments_intersect(a: np.ndarray, b: np.ndarray, c: np.ndarray, d: np.ndarray) -> bool:
    """İki doğru parçasının kesişim testini yapar."""
    return ccw(a, c, d) != ccw(b, c, d) and ccw(a, b, c) != ccw(a, b, d)


def ccw(a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    """Üç noktanın saat yönünün tersinde sıralanıp sıralanmadığını döndürür."""
    return bool((c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0]))


# =============================================================================
# 11. CSV VE GRAFİK YARDIMCILARI
# -----------------------------------------------------------------------------
# Analiz dosyaları bu fonksiyonları kullanarak ortak biçimde CSV yazar ve
# fabrika haritası üzerinde sensör/rota/tahmin görselleri üretir.
# =============================================================================

def write_rows(path: Path, rows: list[dict[str, object]]) -> None:
    """Sözlük listesi biçimindeki analiz sonuçlarını CSV dosyasına yazar."""
    if not rows:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def draw_environment_base(ax, cfg: Config) -> None:
    """Fabrika bölgelerini, dış sınırı ve şişirilmiş engelleri verilen eksene çizer."""
    zones = [
        ((0.0, 20.0), 25.0, 10.0, "#dbeafe", "Parking Docks"),
        ((0.0, 10.0), 25.0, 10.0, "#fde68a", "Battery Loading / Pickup"),
        ((25.0, 10.0), 25.0, 20.0, "#dcfce7", "Main Transit Aisle"),
        ((0.0, 0.0), 50.0, 10.0, "#f3f4f6", "Lower Service Area"),
    ]
    for xy, width, height, color, label in zones:
        ax.add_patch(Rectangle(xy, width, height, facecolor=color, edgecolor="white", linewidth=0.8, alpha=0.62, zorder=1))
        ax.text(xy[0] + width / 2, xy[1] + height / 2, label, ha="center", va="center", fontsize=8, color="#111827", zorder=2)
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
    """Başlangıç, pickup, Gate-2 ve drop-off görev noktalarını harita üzerinde işaretler."""
    markers = [
        (PARKING_START_INDEX, "Start", "#2563eb", "o"),
        (PICKUP_INDEX, "Pickup", "#f59e0b", "D"),
        (GATE2_INDEX, "Gate-2", "#16a34a", "s"),
        (DROPOFF_INDEX, "Drop-off", "#dc2626", "*"),
    ]
    for idx, label, color, marker in markers:
        p = MISSION_POINTS[idx]
        ax.scatter([p[0]], [p[1]], s=90, color=color, marker=marker, zorder=9)
        ax.text(p[0] + 0.35, p[1] + 0.35, label, fontsize=8, color=color, zorder=10)


def plot_case_map_and_error(
    cfg: Config,
    t: np.ndarray,
    true_positions: np.ndarray,
    sensors: np.ndarray,
    result: dict[str, object],
    output_path: Path,
    title: str,
) -> np.ndarray:
    """Tek bir deney için harita+tahmin grafiği ve zaman-hata grafiğini birlikte üretir."""
    ekf = result["ekf"]
    measurements = result["measurements"]
    metrics = result["metrics"]
    estimate = ekf["position"]
    error = np.linalg.norm(estimate - true_positions, axis=1)
    nlos_count = measurements["is_nlos"].sum(axis=1)

    fig, axes = plt.subplots(1, 2, figsize=(14, 6), dpi=150)
    draw_environment_base(axes[0], cfg)
    axes[0].plot(MISSION_POINTS[:, 0], MISSION_POINTS[:, 1], "o--", color="#92400e", linewidth=1.2, label="Mission route")
    axes[0].plot(true_positions[:, 0], true_positions[:, 1], color="black", linewidth=2.0, label="True path")
    axes[0].plot(estimate[:, 0], estimate[:, 1], color="#be123c", linewidth=1.5, label="EKF estimate")
    if np.any(nlos_count > 0):
        axes[0].scatter(true_positions[nlos_count > 0, 0], true_positions[nlos_count > 0, 1], s=10, color="#f97316", alpha=0.55, label="NLOS instant")
    axes[0].scatter(sensors[:, 0], sensors[:, 1], s=70, color="#1d4ed8", zorder=8, label="UWB anchors")
    for idx, sensor in enumerate(sensors, start=1):
        axes[0].text(sensor[0] + 0.35, sensor[1] + 0.35, f"A{idx}", fontsize=8, color="#1e3a8a")
    mark_mission_points(axes[0])
    axes[0].legend(loc="upper right", fontsize=7)
    axes[0].set_title("True path vs EKF estimate")

    axes[1].plot(t, error, color="#047857", linewidth=1.5, label="Position error")
    axes[1].axvspan(10.0, 15.0, color="#f59e0b", alpha=0.18, label="Pickup wait")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("position error [m]")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right", fontsize=8)
    axes[1].set_title(f"RMSE={metrics['rmse_position_m']:.3f} m, mean NLOS={metrics['mean_nlos_anchor_count']:.2f}")

    fig.suptitle(title)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path)
    plt.close(fig)
    return error

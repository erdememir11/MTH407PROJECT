"""
run_02_baseline_constant_noise.py

Bu dosya baseline gürültü senaryosunu çalıştırır.

Baseline senaryoda tüm robot-sensör bağlantıları LOS kabul edilir. Yani fabrika
duvarları ve bölmeleri ölçüm gürültüsünü artırmaz. Böylece sensör geometrisinin
temel etkisi, NLOS etkisi karışmadan incelenebilir.

Üretilen çıktı klasörü:
    outputs/02_baseline_constant_noise/

Bu klasörde:
- 4 sensör için 6 farklı geometriye ait gerçek yol vs EKF tahmini grafikleri,
- 4 sensör hata karşılaştırması,
- 4 sensör RMSE karşılaştırması,
- 4'ten 10'a sensör sayısı karşılaştırması,
- CSV özet tabloları
oluşturulur.
"""

from pathlib import Path

from analysis_pipeline import run_noise_analysis


# =============================================================================
# BASELINE LOS SENARYOSUNU BAŞLATMA
# -----------------------------------------------------------------------------
# Parametre olarak `baseline_constant_los` verildiği için ölçüm üretiminde her
# sensör bağlantısı düşük gürültülü LOS kabul edilir.
# =============================================================================

if __name__ == "__main__":
    run_noise_analysis(
        noise_mode="baseline_constant_los",
        output_dir=Path("outputs") / "02_baseline_constant_noise",
        title="Baseline Constant LOS Noise",
        seed_base=20_000,
    )

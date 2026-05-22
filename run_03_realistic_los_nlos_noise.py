"""
run_03_realistic_los_nlos_noise.py

Bu dosya gerçekçi LOS/NLOS gürültü senaryosunu çalıştırır.

Bu senaryoda robot ile sensör arasındaki doğru parçası şişirilmiş bir duvar
veya engelden geçerse ölçüm NLOS kabul edilir. NLOS ölçümlerde gürültü seviyesi
LOS ölçümlere göre daha yüksektir. Bu nedenle bu senaryo, fabrika ortamındaki
duvar ve bölmelerin UWB/TDOA takip performansına etkisini incelemek için
kullanılır.

Üretilen çıktı klasörü:
    outputs/03_realistic_los_nlos_noise/

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
# GERÇEKÇİ LOS/NLOS SENARYOSUNU BAŞLATMA
# -----------------------------------------------------------------------------
# Parametre olarak `realistic_los_nlos` verildiği için ölçüm üretiminde engel
# kesişimi kontrol edilir. Engel kesişimi varsa ilgili sensör ölçümü NLOS olur.
# =============================================================================

if __name__ == "__main__":
    run_noise_analysis(
        noise_mode="realistic_los_nlos",
        output_dir=Path("outputs") / "03_realistic_los_nlos_noise",
        title="Realistic LOS/NLOS Noise",
        seed_base=30_000,
    )

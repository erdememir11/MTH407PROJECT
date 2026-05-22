"""
run_all_outputs.py

Bu dosya projenin tüm çıktılarını tek komutla üretmek için hazırlanmıştır.

Sırasıyla şu üç çıktı grubunu oluşturur:
1. outputs/01_environment/
   Fabrika ortamı ve beklenen robot hareketi.

2. outputs/02_baseline_constant_noise/
   Tüm ölçümlerin LOS kabul edildiği baseline gürültü senaryosu.

3. outputs/03_realistic_los_nlos_noise/
   Engel kesişimine göre LOS/NLOS ayrımı yapılan gerçekçi gürültü senaryosu.

Proje tesliminde tüm grafik ve CSV dosyalarını güncellemek için bu dosyanın
çalıştırılması yeterlidir.
"""

import generate_01_environment
import run_02_baseline_constant_noise
import run_03_realistic_los_nlos_noise
from pathlib import Path


# =============================================================================
# TÜM ÇIKTILARI SIRAYLA ÜRETME
# -----------------------------------------------------------------------------
# Bu ana blok üç ayrı çalıştırma dosyasının yaptığı işleri tek sırada toplar.
# Böylece ortam, baseline ve realistic senaryoları aynı anda güncellenir.
# =============================================================================

if __name__ == "__main__":
    generate_01_environment.main()
    run_02_baseline_constant_noise.run_noise_analysis(
        noise_mode="baseline_constant_los",
        output_dir=Path("outputs") / "02_baseline_constant_noise",
        title="Baseline Constant LOS Noise",
        seed_base=20_000,
    )
    run_03_realistic_los_nlos_noise.run_noise_analysis(
        noise_mode="realistic_los_nlos",
        output_dir=Path("outputs") / "03_realistic_los_nlos_noise",
        title="Realistic LOS/NLOS Noise",
        seed_base=30_000,
    )

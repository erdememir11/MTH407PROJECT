# Akilli Fabrikalarda Otonom Robot (AGV) Takibi

Bu proje, MTH407 donem projesi dokumanindaki zaman tabanli lokalizasyon ve takip kapsamına gore hazirlanmistir. Senaryo, 2 boyutlu bir fabrika zemininde hareket eden AGV'nin sabit UWB anchor antenlerinden uretilen TDOA olcumleriyle takip edilmesini modeller.

## Icerik

- `run_agv_tdoa_ekf.m`: Simulasyon, TDOA olcum uretimi, LSE ilklendirme, EKF takip ve sensor geometrisi analizini iceren ana MATLAB dosyasi.
- `outputs/tracking_results.csv`: Zaman, gercek konum, tahmin konumu, hata ve NLOS anchor sayisi.
- `outputs/main_metrics.csv`: Ana senaryo RMSE/ortalama/maksimum hata metrikleri.
- `outputs/geometry_analysis.csv`: Iyi ve zayif sensor geometrisi icin Monte Carlo RMSE ve kosul sayisi karsilastirmasi.
- `outputs/*.png`: Rapor icin kullanilabilecek grafikler.

`outputs` klasoru script calistirildiginda otomatik olusur.

## Calistirma

MATLAB icinde:

```matlab
run('run_agv_tdoa_ekf.m')
```

Komut satirinda:

```powershell
matlab -batch "run('run_agv_tdoa_ekf.m')"
```

## Model Ozeti

- Durum vektoru: `[x, y, vx, vy]^T`
- Hareket modeli: sabit hiz modeli + ivme surec gurultusu
- Olcum modeli: referans anchor'a gore TDOA menzil farki
- Ilklendirme: ilk TDOA olcumlerinden Gauss-Newton LSE ile konum, ilk konumlardan hiz tahmini
- Takip algoritmasi: Extended Kalman Filter (EKF)
- Gurultu modeli:
  - LOS/acik alan: dusuk standart sapmali Gauss TOA gurultusu
  - NLOS/engel etkili: daha yuksek standart sapmali Gauss TOA gurultusu

## Rapor Icin Onerilen Basliklar

1. Problem tanimi: Akilli fabrikada AGV takibi ve UWB anchor yerlestirmesi
2. Simulasyon ortami: fabrika boyutu, anchor konumlari, engeller ve AGV yoruengesi
3. TDOA olcum modeli: menzil farki denklemi, referans anchor secimi ve gurultu kovaryansi
4. LSE ilklendirme: ilk konum ve kovaryans tahmini
5. EKF tasarimi: durum modeli, olcum Jacobian'i, surec/olcum kovaryanslari
6. Deneyler: iyi geometri, zayif geometri, LOS/NLOS gurultu etkisi
7. Sonuclar: gercek-tahmin konum grafikleri, hata grafikleri, RMSE tablolari

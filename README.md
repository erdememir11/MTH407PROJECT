# Akilli Fabrikalarda Otonom Robot (AGV) Takibi

Bu proje, MTH407 donem projesi dokumanindaki zaman tabanli lokalizasyon ve takip kapsamina gore hazirlanmistir. Senaryo, 2 boyutlu bir fabrika zemininde hareket eden AGV'nin sabit UWB anchor antenlerinden uretilen TDOA olcumleriyle takip edilmesini modeller.

## Icerik

- `agv_tdoa_ekf.py`: Simulasyon, TDOA olcum uretimi, LSE ilklendirme, EKF takip ve sensor geometrisi analizini iceren ana Python dosyasi.
- `requirements.txt`: Gerekli Python kutuphaneleri.
- `run_agv_tdoa_ekf.m`: Ayni modelin onceki MATLAB surumu.
- `outputs/tracking_results.csv`: Zaman, gercek konum, tahmin konumu, hata ve NLOS anchor sayisi.
- `outputs/main_metrics.csv`: Ana senaryo RMSE/ortalama/maksimum hata metrikleri.
- `outputs/geometry_analysis.csv`: Iyi ve zayif sensor geometrisi icin Monte Carlo RMSE ve kosul sayisi karsilastirmasi.
- `outputs/*.png`: Rapor icin kullanilabilecek grafikler.

`outputs` klasoru script calistirildiginda otomatik olusur.

## Calistirma

Gerekli kutuphaneleri kurmak icin:

```powershell
pip install -r requirements.txt
```

Projeyi calistirmak icin:

```powershell
python agv_tdoa_ekf.py
```

Bu makinede Python PATH uzerinde gorunmuyorsa Anaconda kurulumu ile su komut da kullanilabilir:

```powershell
C:\ANACONDA\python.exe agv_tdoa_ekf.py
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

## Simulasyon Ortami Detaylari

Fabrika zemini 2 boyutlu dikdortgen bir alan olarak modellenmistir.

- Fabrika boyutu: `42 m x 26 m`
- Koordinat sistemi:
  - Sol alt kose: `(0, 0)`
  - Sag ust kose: `(42, 26)`
  - Birimler metre cinsindendir.
- AGV, bu alan icinde hareket eder ve sinir disina cikarsa hizinin ilgili bileseni ters cevrilerek alana geri tutulur.

### UWB Anchor / Sensor Konumlari

Ana senaryoda 6 adet sabit UWB anchor kullanilmistir. Bunlar fabrika duvarlarina veya kolonlara yerlestirilmis sabit antenleri temsil eder.

| Anchor | x [m] | y [m] | Yorum |
|---|---:|---:|---|
| A1 | 1.5 | 1.5 | Sol alt duvar/koseye yakin, TDOA referans anchor |
| A2 | 40.0 | 1.2 | Sag alt duvara yakin |
| A3 | 41.0 | 24.5 | Sag ust duvara yakin |
| A4 | 1.2 | 24.0 | Sol ust duvara yakin |
| A5 | 20.5 | 3.2 | Alt hatta/kolona yakin orta anchor |
| A6 | 31.5 | 17.5 | Ic bolgede kolon uzeri anchor |

Kodda bu konumlar `agv_tdoa_ekf.py` icindeki `SENSORS_GOOD` degiskenindedir.

TDOA icin referans anchor `A1` secilmistir. Yani olcumler su mantikla uretilir:

```text
z_i = mesafe(AGV, A_i) - mesafe(AGV, A1) + gurultu
```

Burada `i = A2, A3, A4, A5, A6` icindir. Bu nedenle her zaman adiminda 5 adet TDOA menzil farki olcumu vardir.

### Engel / Makine Yerlesimi

Fabrika icindeki sinyal bozucu engeller dikdortgen bolgeler olarak modellenmistir. Bu engeller makine, raf, kolon grubu veya metal uretim hatti gibi dusunulebilir.

Her engel `[x, y, genislik, yukseklik]` seklinde tanimlanmistir.

| Engel | x [m] | y [m] | Genislik [m] | Yukseklik [m] | Yorum |
|---|---:|---:|---:|---:|---|
| E1 | 8.0 | 6.0 | 5.5 | 4.0 | Sol-alt bolgede makine adasi |
| E2 | 18.0 | 10.0 | 7.0 | 3.5 | Orta bolgede uretim hatti |
| E3 | 29.0 | 5.0 | 4.0 | 8.0 | Sag-alt bolgede metal/raf engeli |
| E4 | 12.0 | 18.0 | 10.0 | 3.0 | Ust bolgede uzun hat engeli |

Kodda bu bolgeler `OBSTACLES` degiskenindedir.

Bir AGV-anchor dogrusu bu dikdortgenlerden biriyle kesismesi halinde o anchor icin olcum `NLOS` kabul edilir. NLOS durumda daha yuksek Gauss gurultusu kullanilir.

## Gurultu Modeli

Proje gercekci UWB/TDOA davranisini taklit etmek icin iki seviyeli Gauss gurultusu kullanir.

| Durum | TOA standart sapmasi | Menzil karsiligi | Anlam |
|---|---:|---:|---|
| LOS / acik alan | `0.20 / c` saniye | yaklasik `0.20 m` | Anchor ile AGV arasinda engel yok |
| NLOS / engelli hat | `1.20 / c` saniye | yaklasik `1.20 m` | Sinyal engelden etkileniyor |

`c = 299792458 m/s` elektromanyetik dalga yayilma hizidir.

TDOA olcumu iki TOA farkindan olustugu icin kovaryans hesabi su sekilde yapilir:

```text
Var(z_i) = c^2 * (sigma_i^2 + sigma_ref^2)
```

Burada `sigma_i`, ilgili anchor'un TOA gurultusu; `sigma_ref`, referans anchor olan A1'in TOA gurultusudur.

## AGV Hareket Modeli

AGV'nin gercek yoruengesi yapay olarak uretilir. Baslangic durumu:

```text
x0 = 4.0 m
y0 = 4.0 m
vx0 = 1.05 m/s
vy0 = 0.55 m/s
```

Simulasyon ayarlari:

- Zaman adimi: `dt = 0.2 s`
- Adim sayisi: `260`
- Toplam simulasyon suresi: `51.8 s`
- Maksimum hiz: `1.8 m/s`

Hareket, sabit hiz modeline eklenen yavas degisen sinuzoidal ivmelerle uretilir. Bu sayede AGV tamamen duz bir cizgide gitmez; fabrika icinde daha gercekci, kivrimli bir yol izler.

EKF tarafinda kullanilan surec modeli:

```text
x_k = x_{k-1} + dt * vx_{k-1}
y_k = y_{k-1} + dt * vy_{k-1}
vx_k = vx_{k-1}
vy_k = vy_{k-1}
```

Surec gurultusu ivme belirsizligi olarak modellenir:

```text
sigma_accel = 0.45 m/s^2
```

## LSE Ilklendirme Detaylari

EKF'nin baslangic konumunu elde etmek icin ilk TDOA olcumleri kullanilir.

- Ilk `8` zaman adimindaki TDOA olcumlerinden konum tahmini yapilir.
- Her zaman adimi icin Gauss-Newton LSE uygulanir.
- Ilk konum, ilk LSE konum tahmininden alinir.
- Ilk hiz, ilk 8 LSE konumuna dogrusal egri uydurularak bulunur.
- Baslangic kovaryansi, LSE Jacobian matrisinden yaklasik olarak hesaplanir.

LSE'nin cozmeye calistigi problem:

```text
min_p || z - h(p) ||_R
```

Burada:

- `p = [x, y]^T` hedef konumudur.
- `z` TDOA menzil farki olcumleridir.
- `h(p)` anchor geometrisinden hesaplanan teorik menzil farklaridir.
- `R` olcum gurultusu kovaryans matrisidir.

## EKF Takip Detaylari

EKF durum vektoru:

```text
X = [x, y, vx, vy]^T
```

Her zaman adiminda iki islem yapilir:

1. Tahmin:
   - Sabit hiz modeliyle AGV'nin yeni durumu tahmin edilir.
   - Surec kovaryansi `Q`, ivme gurultusundan uretilir.
2. Guncelleme:
   - Tahmin edilen konuma gore TDOA olcum fonksiyonu hesaplanir.
   - Olcum fonksiyonu dogrusal olmadigi icin Jacobian matrisi kullanilir.
   - Kalman kazanci ile konum ve hiz tahmini guncellenir.

Olcum fonksiyonu:

```text
h_i(x, y) = ||p - A_i|| - ||p - A_ref||
```

Jacobian:

```text
dh_i/dp = (p - A_i) / ||p - A_i|| - (p - A_ref) / ||p - A_ref||
```

## Sensor Geometrisi Analizi

Proje dokumaninda sensör geometrisinin performansa etkisinin incelenmesi isteniyor. Bunun icin iki geometri karsilastirildi.

### Iyi Geometri

Ana senaryodaki `SENSORS_GOOD` dizilimidir. Anchorlar fabrika alanini cevreleyecek sekilde farkli kose ve ic bolgelere dagitilmistir. Bu, TDOA icin daha iyi geometrik gozlenebilirlik saglar.

### Zayif Geometri

Karsilastirma icin kullanilan zayif geometride anchorlar neredeyse ayni alt duvar hattina dizilmistir:

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 2.0 | 1.0 |
| A2 | 9.0 | 1.2 |
| A3 | 16.0 | 0.9 |
| A4 | 23.0 | 1.1 |
| A5 | 30.0 | 0.8 |
| A6 | 38.0 | 1.0 |

Bu geometri, hedefin ozellikle y yonundeki konumunu ayirmayi zorlastirir. Bu nedenle `cond(H^T H)` degeri yukselir ve RMSE artar.

Son calistirmada uretilen karsilastirma:

| Geometri | Ortalama RMSE [m] | RMSE std [m] | Ortalama kosul sayisi |
|---|---:|---:|---:|
| Iyi geometri | 0.415570 | 0.039767 | 3.982372 |
| Zayif geometri | 1.872735 | 1.706336 | 36.766317 |

## Revize Etmek Icin Nereler Degistirilmeli?

Projeyi kendi senaryona gore degistirmek icin en onemli yerler `agv_tdoa_ekf.py` dosyasinin basindadir.

- Fabrika boyutu:
  - `Config.factory_size`
- Zaman adimi ve simulasyon suresi:
  - `Config.dt`
  - `Config.n_steps`
- UWB anchor konumlari:
  - `SENSORS_GOOD`
  - `SENSORS_POOR`
- Engel/makine yerlesimi:
  - `OBSTACLES`
- LOS/NLOS gurultu seviyeleri:
  - `Config.sigma_toa_los`
  - `Config.sigma_toa_nlos`
- EKF surec gurultusu:
  - `Config.sigma_accel`
- LSE iterasyon sayisi:
  - `Config.lse_iterations`
- Sensor geometrisi Monte Carlo tekrar sayisi:
  - `Config.geometry_monte_carlo_runs`

Ornegin daha gurultulu bir fabrika ortami icin `sigma_toa_nlos` menzil karsiligi `1.20 m` yerine `2.00 m` yapilabilir:

```python
return 2.00 / self.c
```

Daha buyuk bir fabrika icin:

```python
factory_size: tuple[float, float] = (80.0, 50.0)
```

## Rapor Icin Onerilen Basliklar

1. Problem tanimi: Akilli fabrikada AGV takibi ve UWB anchor yerlestirmesi
2. Simulasyon ortami: fabrika boyutu, anchor konumlari, engeller ve AGV yorungesi
3. TDOA olcum modeli: menzil farki denklemi, referans anchor secimi ve gurultu kovaryansi
4. LSE ilklendirme: ilk konum ve kovaryans tahmini
5. EKF tasarimi: durum modeli, olcum Jacobian'i, surec/olcum kovaryanslari
6. Deneyler: iyi geometri, zayif geometri, LOS/NLOS gurultu etkisi
7. Sonuclar: gercek-tahmin konum grafikleri, hata grafikleri, RMSE tablolari

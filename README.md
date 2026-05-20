# Fabrika Ortaminda Batarya Yerlestirme Robotu Takibi

Bu proje, MTH407 donem projesi kapsaminda UWB/TDOA olcumleri ile bir mobil robotun 2D fabrika ortaminda takip edilmesini modeller. Guncel senaryo, Parking Docks bolgesinden baslayan robotun Battery Loading / Pickup noktasinda bataryayi almasi ve drop-off noktasina devam etmesidir.

Ana algoritma yapisi:

- Gercek robot hareketi: waypoint tabanli sabit hiz modeli
- Olcum modeli: UWB anchor antenleri ile TDOA menzil farki
- Baslangic tahmini: LSE / Gauss-Newton
- Takip algoritmasi: EKF
- Performans analizi: 4 sensor icin 3 geometri ve 4-10 sensor sayisi karsilastirmasi

## Dosyalar

- `agv_tdoa_ekf.py`: Ana Python simulasyon, LSE, EKF ve analiz kodu.
- `requirements.txt`: Gerekli Python paketleri.
- `factory_geometry_visual.py`: Sadece fabrika geometrisini cizen aciklamali Python kodu.
- `outputs/tracking_results.csv`: Zaman, gercek konum, tahmin konumu, hata ve NLOS sensor sayisi.
- `outputs/main_metrics.csv`: Ana takip senaryosu hata metrikleri.
- `outputs/four_sensor_geometry_analysis.csv`: 4 sensorlu 3 farkli geometri karsilastirmasi.
- `outputs/sensor_count_analysis.csv`: 4-10 sensor sayisi karsilastirmasi.
- `outputs/factory_geometry_only.png`: Sadece fabrika geometrisi, bolgeler, kapilar ve engeller.
- `outputs/battery_robot_tracking_map.png`: Fabrika haritasi, waypoint rotasi, engeller, sensorler ve EKF takibi.
- `outputs/tracking_time_series.png`: x/y konumu ve zamanla konum hatasi.
- `outputs/geometry_and_sensor_count_analysis.png`: Geometri ve sensor sayisi analiz grafigi.

## Calistirma

Gerekli paketler:

```powershell
pip install -r requirements.txt
```

Bu makinede Python PATH uzerinde gorunmuyorsa Anaconda ile:

```powershell
C:\ANACONDA\python.exe agv_tdoa_ekf.py
```

Normal Python kurulumunda:

```powershell
python agv_tdoa_ekf.py
```

## Calisma Ortami

PDF dokumanindaki yeni fabrika modeli kullanildi.

Koordinat sistemi:

- Orijin: sol-alt kose
- x ekseni: saga dogru
- y ekseni: yukari dogru
- Birim: metre

Alan sinirlari:

```text
0 <= x <= 50 m
0 <= y <= 30 m
```

Baslangic, pickup ve hedef:

| Nokta | Koordinat [m] | Anlam |
|---|---:|---|
| S | `(5, 25)` | Parking Docks baslangici |
| P | `(5, 15)` | Battery Loading / Pickup noktasi |
| G | `(45, 10)` | Batarya yerlestirme / drop-off hedefi |

Semantik bolgeler:

| Bolge | Koordinat araligi | Rol |
|---|---|---|
| Parking Docks | `0 <= x <= 25`, `20 <= y <= 30` | Robotun goreve basladigi park/dock bolgesi |
| Battery Loading / Pickup | `0 <= x <= 25`, `10 <= y <= 20` | Robotun bataryayi teslim aldigi bolge |
| Main transit aisle | `25 <= x <= 50`, `10 <= y <= 30` | Ana ulasim koridoru |
| Drop-off | `x = 45`, `y = 10` | Batarya yerlestirme noktasi |

## Robot Hareket Rotasi

Robot hareketi artik cok waypointli serbest rota yerine gorev fazlariyla tanimlanir. Simulasyon robotun Parking Dock noktasindan cikmasiyla baslar, pickup noktasinda batarya yukleme icin 5 saniye bekler ve drop-off noktasina ulasinca sonlandirilir.

Kodda kullanilan gorev noktalar:

| Nokta | Koordinat [m] | Aciklama |
|---|---:|---|
| M0 | `(5, 25)` | Baslangic / Parking Docks |
| M1 | `(5, 15)` | Pickup: bataryayi teslim alma ve 5 saniye bekleme |
| M2 | `(25, 18)` | Gate-2 acikligi / main aisle girisi |
| M3 | `(45, 10)` | Drop-off hedefi |

Hareket fazlari:

| Faz | Hareket | Hiz | Aciklama |
|---|---|---:|---|
| 1 | `(5,25) -> (5,15)` | `1.0 m/s` | Gate-1 uzerinden tamamen dikey hareket |
| 2 | `(5,15)` uzerinde bekleme | `0.0 m/s` | 5 saniyelik batarya yukleme islemi |
| 3 | `(5,15) -> (25,18)` | `1.0 m/s` | Gate-2 acikligindan ana koridora cikis |
| 4 | `(25,18) -> (45,10)` | `1.0 m/s` | Ana koridordan drop-off'a en az yon degisimiyle ilerleme |

Hareket parametreleri:

| Parametre | Deger |
|---|---:|
| Zaman adimi `dt` | `0.5 s` |
| Nominal hiz | `1.0 m/s` |
| Pickup bekleme suresi | `5.0 s` |
| Toplam simulasyon suresi | `57.5 s` |
| Simulasyon bitis kosulu | Drop-off noktasina ulasma |

EKF durum vektoru:

```text
X = [x, y, vx, vy]^T
```

Sabit hiz modeli:

```text
x_k = x_{k-1} + dt * vx_{k-1}
y_k = y_{k-1} + dt * vy_{k-1}
vx_k = vx_{k-1}
vy_k = vy_{k-1}
```

Surec gurultusu:

```text
sigma_accel = 0.35 m/s^2
```

## Engel ve Kapi Modeli

Robot noktasal kabul edilir; buna karsilik duvar ve bolmeler robot yaricapi ve guvenlik mesafesi kadar sisirilir.

Robot ve guvenlik parametreleri:

| Parametre | Deger |
|---|---:|
| Robot yaricapi `r_robot` | `0.35 m` |
| Guvenlik mesafesi `d_margin` | `0.25 m` |
| Toplam guvenli sisirme `d_safe` | `0.60 m` |
| Ham duvar kalinligi | `0.20 m` |
| Sisirilmis yari kalinlik | `0.70 m` |

Ham bolme/duvar segmentleri:

| Kod | Geometri | Aciklama |
|---|---|---|
| B0 | Dis sinir: `x=0`, `x=50`, `y=0`, `y=30` | Robot alan disina cikamaz |
| B1 | `y=20`, `0 <= x <= 25` | Parking Docks ile Battery Loading / Pickup arasindaki bolme |
| B2 | `x=25`, `10 <= y <= 30` | Sol bolgeler ile main transit aisle arasindaki bolme |
| B3 | `y=10`, `0 <= x <= 40` | Alt bolgeyi ayiran yatay sinir |
| B4 | `x=40`, `0 <= y <= 10` | Drop-off tarafindaki dikey sinir |

Kodda kapilar/acikliklar nedeniyle B1 ve B2 parcalara ayrilmistir.

Sisirilmis yasak dikdortgenler:

| Engel | Dikdortgen `(xmin, ymin, xmax, ymax)` | Neden |
|---|---|---|
| B1-left | `(0.0, 19.3, 4.0, 20.7)` | Gate-1 solundaki B1 parcasi |
| B1-right | `(6.0, 19.3, 25.0, 20.7)` | Gate-1 sagindaki B1 parcasi |
| B2-lower | `(24.3, 10.0, 25.7, 16.6)` | Gate-2 alt parcasi; B3 ile arasindaki gereksiz bosluk kaldirildi |
| B2-upper | `(24.3, 19.0, 25.7, 30.0)` | Gate-2 ust parcasi |
| B3 | `(0.0, 9.3, 40.0, 10.7)` | Alt yatay bolme |
| B4 | `(39.3, 0.0, 40.7, 10.0)` | Drop-off dikey bolmesi |

Kapi/acikliklar:

| Kapi | Koordinat tanimi | Kod etkisi |
|---|---|---|
| Gate-1 | `x ~= 5`, `y = 20` | B1, `x=4..6` araliginda acik birakildi |
| Gate-2 | `x = 25`, `16.6 <= y <= 19.0` | B2, bu y araliginda acik birakildi |
| Drop-off yaklasimi | `x > 40`, `y = 10` | B3 yalnizca `x <= 40` icin engel kabul edildi |

## TDOA Olcum Modeli

Her UWB anchor sinyal varis zamani uzerinden olcum uretir. TDOA modelinde mutlak TOA yerine referans anchor'a gore zaman farki kullanilir.

Referans anchor:

```text
A1
```

Menzil farki olcum modeli:

```text
z_i = ||p - A_i|| - ||p - A_ref|| + v_i
```

Burada:

- `p = [x, y]^T` robot konumu
- `A_i` i'inci UWB anchor konumu
- `A_ref` referans anchor konumu
- `v_i` TDOA menzil farki gurultusu

4 sensor kullanildiginda 1 referans + 3 fark olcumu vardir. Genel olarak `M` sensor icin olcum boyutu `M-1` olur.

## Gurultu ve NLOS Modeli

Fabrika icindeki bolmeler sinyal bozucu engel kabul edilir. Robot-anchor dogrusu sisirilmis engellerden biriyle kesisse o anchor icin NLOS olcum uretilir.

| Durum | TOA standart sapmasi | Menzil karsiligi | Anlam |
|---|---:|---:|---|
| LOS / acik gorus | `0.25 / c` saniye | `0.25 m` | Robot-anchor dogrusu engelden gecmiyor |
| NLOS / engelli hat | `1.50 / c` saniye | `1.50 m` | Robot-anchor dogrusu engelden etkileniyor |

TDOA kovaryans hesabi:

```text
Var(z_i) = c^2 * (sigma_i^2 + sigma_ref^2)
```

Bu nedenle referans anchor'un NLOS olmasi tum TDOA olcumlerini olumsuz etkiler.

## LSE Ilklendirme

EKF baslangic durumu ilk TDOA olcumlerinden bulunur.

Adimlar:

1. Ilk 8 zaman adimi kullanilir.
2. Her zaman adimi icin Gauss-Newton LSE ile `[x, y]` konumu tahmin edilir.
3. Ilk konum, ilk LSE sonucundan alinir.
4. Ilk hiz, ilk 8 LSE konumuna dogrusal egri uydurularak hesaplanir.
5. Baslangic kovaryansi, LSE Jacobian yaklasimindan uretilir.

LSE problemi:

```text
min_p || z - h(p) ||_R
```

## EKF Takip Modeli

EKF iki asamada calisir.

Tahmin:

```text
X_pred = F X_prev
P_pred = F P_prev F^T + Q
```

Guncelleme:

```text
y = z - h(X_pred)
S = H P_pred H^T + R
K = P_pred H^T S^-1
X = X_pred + K y
P = (I - K H) P_pred (I - K H)^T + K R K^T
```

TDOA olcum Jacobian'i:

```text
dh_i/dp = (p - A_i) / ||p - A_i|| - (p - A_ref) / ||p - A_ref||
```

## 4 Sensor Icin 3 Geometri

Bir sonraki proje adimi icin 4 sensor sayisi sabit tutulup 3 farkli yerlesim test edildi.

### G1 Corner Coverage

Alan koselerine yakin dengeli yerlesimdir. Bu geometri ana senaryo olarak kullanilir. A1, kullanici revizyonuna gore B3 duvarinin sol ucuna alinmistir.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 0.0 | 10.0 |
| A2 | 49.0 | 1.0 |
| A3 | 49.0 | 29.0 |
| A4 | 1.0 | 29.0 |

### G2 Task Oriented

Robotun gorev rotasina ve drop-off alanina daha yakin anchorlar icerir. Pratikte bu geometri bazi rota bolumlerinde referans veya diger anchorlarin NLOS olmasina daha yatkindir.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 0.0 | 10.0 |
| A2 | 24.0 | 27.5 |
| A3 | 28.0 | 15.5 |
| A4 | 48.0 | 11.0 |

### G3 Poor Same Wall

Kotu geometri ornegidir. A1 B3 duvarinin sol ucunda sabit tutulurken diger anchorlar alt hatta yakin secilmistir. Bu dizilim TDOA icin y yonundeki bilgiyi zayiflatir.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 0.0 | 10.0 |
| A2 | 17.0 | 2.0 |
| A3 | 33.0 | 2.0 |
| A4 | 48.0 | 2.0 |

Son calistirma sonucu:

| Geometri | Ortalama RMSE [m] | RMSE std [m] | Ortalama kosul sayisi |
|---|---:|---:|---:|
| G1 corner coverage | 0.832 | 0.077 | 4.796 |
| G2 task oriented | 1.387 | 0.286 | 21.713 |
| G3 poor same wall | 2.829 | 2.038 | 42.051 |

Yorum: A1'in B3 duvarinin sol ucuna alinmasindan sonra G1 hala en dusuk ortalama RMSE'ye sahiptir. G2 onceki yerlesime gore belirgin sekilde iyilesmistir. G3 ise artik onceki kadar kotu degildir; cunku A1'in `(0,10)` konumuna alinmasi, tamamen alt hatta yigilmis referans geometrisini kismen bozarak kosullamayi iyilestirmistir.

## A1 Revizyonu Once / Sonra Karsilastirmasi

A1 revizyonundan once referans anchor G1 ve sensor sayisi analizinde `(1,1)` konumundaydi. Revizyondan sonra A1, B3 duvarinin en sol noktasi olan `(0,10)` konumuna tasindi.

Iki farkli sonuc tipi vardir:

- Ana takip RMSE: tek bir sabit rastgele tohumla calisan ana senaryo sonucudur.
- Ortalama RMSE: Monte Carlo tekrarlarinin ortalamasidir; geometri karsilastirmasi icin daha guvenilir yorum budur.

| Metrik | A1 once `(1,1)` | A1 sonra `(0,10)` | Yorum |
|---|---:|---:|---|
| Ana takip RMSE | 0.692 | 0.675 | Tekil ana kosuda biraz iyilesti |
| G1 4-sensor ortalama RMSE | 0.814 | 0.832 | Monte Carlo ortalamasinda biraz kotulesti |
| G2 4-sensor ortalama RMSE | 1.860 | 1.387 | Belirgin iyilesti |
| G3 4-sensor ortalama RMSE | 14.607 | 2.829 | Cok ciddi iyilesti |
| 8 sensor ortalama RMSE | 0.737 | 0.743 | Neredeyse ayni, cok az kotulesti |
| 10 sensor ortalama RMSE | 0.744 | 0.744 | Pratikte ayni kaldi |

Sonuc: "A1'i B3 sol ucuna almak her durumda daha iyi" demek dogru olmaz. Ana tekil takip senaryosunda iyilesme var, G2 ve G3 geometrilerinde belirgin iyilesme var; fakat G1 corner coverage ve 8 sensor ortalamasi cok az kotulesiyor. Rapor icin en dogru yorum: A1'in B3 sol ucuna alinmasi bazi kotu geometrileri stabilize ediyor, fakat dengeli kose geometrisinde kucuk bir bedel olusturuyor.

## Detayli Grafik Dosyalari

4 sensorlu geometri ana inceleme alani oldugu icin A1 revizyonunun oncesi ve sonrasi ayri klasorlere ayrildi. Bu grafikler `generate_analysis_figures.py` ile uretilir.

Calistirma:

```powershell
C:\ANACONDA\python.exe generate_analysis_figures.py
```

### A1 Degismeden Once

Klasor:

```text
outputs/analysis_4_sensor_a1_before
```

Icerik:

| Dosya | Aciklama |
|---|---|
| `G1_corner_coverage_map_and_error.png` | Eski A1 ile G1 harita + hata grafigi |
| `G2_task_oriented_map_and_error.png` | Eski A1 ile G2 harita + hata grafigi |
| `G3_poor_same_wall_map_and_error.png` | Eski A1 ile G3 harita + hata grafigi |
| `summary_rmse_by_geometry.png` | Eski A1 icin 3 geometrinin ozet RMSE grafigi |
| `summary.csv` | Eski A1 icin sayisal metrikler |

### A1 B3 Sol Ucuna Alindiktan Sonra

Klasor:

```text
outputs/analysis_4_sensor_a1_after
```

Icerik:

| Dosya | Aciklama |
|---|---|
| `G1_corner_coverage_map_and_error.png` | Yeni A1 ile G1 harita + hata grafigi |
| `G2_task_oriented_map_and_error.png` | Yeni A1 ile G2 harita + hata grafigi |
| `G3_poor_same_wall_map_and_error.png` | Yeni A1 ile G3 harita + hata grafigi |
| `summary_rmse_by_geometry.png` | Yeni A1 icin 3 geometrinin ozet RMSE grafigi |
| `summary.csv` | Yeni A1 icin sayisal metrikler |

### A1 Once / Sonra Ortak Ozet

Dosya:

```text
outputs/analysis_4_sensor_a1_before_after_summary.png
```

Bu grafik, 4 sensorlu G1/G2/G3 geometrilerinde A1 revizyonu oncesi ve sonrasini yan yana karsilastirir.

### 4'ten 10'a Sensor Sayisi Analizi

Klasor:

```text
outputs/analysis_sensor_count_4_to_10
```

Icerik:

| Dosya | Aciklama |
|---|---|
| `sensor_count_04_map_and_error.png` | 4 sensorlu harita + hata grafigi |
| `sensor_count_05_map_and_error.png` | 5 sensorlu harita + hata grafigi |
| `sensor_count_06_map_and_error.png` | 6 sensorlu harita + hata grafigi |
| `sensor_count_07_map_and_error.png` | 7 sensorlu harita + hata grafigi |
| `sensor_count_08_map_and_error.png` | 8 sensorlu harita + hata grafigi |
| `sensor_count_09_map_and_error.png` | 9 sensorlu harita + hata grafigi |
| `sensor_count_10_map_and_error.png` | 10 sensorlu harita + hata grafigi |
| `summary_sensor_count_effect.png` | Sensor sayisi arttikca RMSE, kosul sayisi ve NLOS degisimi |
| `summary.csv` | 4-10 sensor icin sayisal metrikler |

Not: Bu detayli grafik betigi, ana simülasyon betiginden bagimsiz sabit rastgele tohumlar kullanarak yeniden Monte Carlo hesaplar. Bu nedenle README'deki genel ozet tablo ile bu klasorlerdeki `summary.csv` degerleri arasinda kucuk sayisal farklar olabilir. Analiz yaparken ayni tablo/grafik ailesi icindeki degerler kendi arasinda karsilastirilmalidir.

## Sensor Sayisi Analizi

Sensor sayisi 4'ten baslatilip 10'a kadar arttirildi. Maksimum sensor sayisi bu asama icin `10` secildi.

Bu secimin gerekcesi:

- 50x30 m gibi orta boy bir fabrika plani icin 4 anchor TDOA'nin teorik minimum pratik baslangicidir.
- 6-8 anchor genellikle kapsama ve dayaniklilik icin yeterli iyilesme saglar.
- 10 anchor sonrasi ek maliyet artar, fakat bu harita olceginde beklenen ek kazanc azalir.
- Daha fazla anchor, NLOS anchor sayisini da artirabilecegi icin performans her zaman monoton iyilesmek zorunda degildir.

Kullanilan 10 sensorluk maksimum havuz:

| Sira | x [m] | y [m] | Aciklama |
|---|---:|---:|---|
| 1 | 0.0 | 10.0 | B3 duvarinin sol ucu, referans A1 |
| 2 | 49.0 | 1.0 | Sag alt kose |
| 3 | 49.0 | 29.0 | Sag ust kose |
| 4 | 1.0 | 29.0 | Sol ust kose |
| 5 | 25.0 | 29.0 | Ust orta |
| 6 | 49.0 | 15.0 | Sag orta |
| 7 | 1.0 | 15.0 | Sol orta |
| 8 | 25.0 | 1.0 | Alt orta |
| 9 | 25.0 | 18.0 | Gate-2 / main aisle yakininda kolon anchor |
| 10 | 45.0 | 10.0 | Drop-off yakin anchor |

Son calistirma sonucu:

| Sensor sayisi | Ortalama RMSE [m] |
|---:|---:|
| 4 | 0.843 |
| 5 | 0.796 |
| 6 | 0.772 |
| 7 | 0.795 |
| 8 | 0.743 |
| 9 | 0.759 |
| 10 | 0.744 |

Yorum: En iyi ortalama sonuc bu calistirmada 8 sensor civarinda goruldu. Daha fazla sensor eklemek her zaman monoton iyilesme saglamaz; cunku yeni eklenen anchorlarin belirli rota bolumlerinde NLOS olcumu uretmesi ve TDOA referans yapisinin dogrusal olmayan etkileri performansi etkiler. Bu bulgu raporda "sensor sayisi artisi geometri ve NLOS kosullariyla birlikte degerlendirilmelidir" seklinde yorumlanabilir.

## Revizyon Icin Kodda Bakilacak Yerler

Temel ortam parametreleri `Config` sinifindadir:

- `factory_size`: fabrika boyutu
- `dt`: zaman adimi
- `nominal_speed`: robot hizi
- `pickup_wait_time`: pickup noktasindaki batarya yukleme bekleme suresi
- `robot_radius`, `safety_margin`: engel sisirme parametreleri
- `sigma_toa_los`, `sigma_toa_nlos`: UWB olcum gurultuleri
- `sigma_accel`: EKF surec gurultusu
- `monte_carlo_runs`: analiz tekrar sayisi

Gorev noktasi rotasi:

- `MISSION_POINTS`

4 sensor geometrileri:

- `SENSOR_GEOMETRIES_4`

Sensor sayisi analizinde kullanilan maksimum havuz:

- `SENSOR_POOL_MAX_10`

Engel ve kapi modeli:

- `inflated_obstacles(cfg)`

## Rapor Icin Onerilen Basliklar

1. Problem tanimi: batarya yerlestirme gorevli fabrika robotu takibi
2. Fabrika ortami: 50x30 m koordinat sistemi, bolgeler, kapilar ve engeller
3. Robot hareket modeli: waypoint tabanli sabit hizli hareket
4. UWB/TDOA olcum modeli ve LOS/NLOS gurultu modeli
5. LSE ile baslangic durum tahmini
6. EKF takip algoritmasi
7. 4 sensorlu 3 geometri analizi
8. 4-10 sensor sayisi analizi
9. Sonuclar ve yorum: RMSE, NLOS etkisi, sensor geometrisinin onemi

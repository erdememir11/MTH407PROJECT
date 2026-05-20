# Fabrika Ortaminda Batarya Yerlestirme Robotu Takibi

Bu proje, MTH407 donem projesi kapsaminda UWB/TDOA olcumleri ile bir mobil robotun 2D fabrika ortaminda takip edilmesini modeller. Guncel senaryo, warehouse bolgesinden baslayip batarya/drop-off noktasina giden bir robotun izlenmesidir.

Ana algoritma yapisi:

- Gercek robot hareketi: waypoint tabanli sabit hiz modeli
- Olcum modeli: UWB anchor antenleri ile TDOA menzil farki
- Baslangic tahmini: LSE / Gauss-Newton
- Takip algoritmasi: EKF
- Performans analizi: 4 sensor icin 3 geometri ve 4-10 sensor sayisi karsilastirmasi

## Dosyalar

- `agv_tdoa_ekf.py`: Ana Python simülasyon, LSE, EKF ve analiz kodu.
- `requirements.txt`: Gerekli Python paketleri.
- `outputs/tracking_results.csv`: Zaman, gercek konum, tahmin konumu, hata ve NLOS sensor sayisi.
- `outputs/main_metrics.csv`: Ana takip senaryosu hata metrikleri.
- `outputs/four_sensor_geometry_analysis.csv`: 4 sensorlu 3 farkli geometri karsilastirmasi.
- `outputs/sensor_count_analysis.csv`: 4-10 sensor sayisi karsilastirmasi.
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

Baslangic ve hedef:

| Nokta | Koordinat [m] | Anlam |
|---|---:|---|
| S | `(5, 25)` | Warehouse / battery loading baslangici |
| G | `(45, 10)` | Batarya yerlestirme / drop-off hedefi |

Semantik bolgeler:

| Bolge | Koordinat araligi | Rol |
|---|---|---|
| Warehouse zone | `0 <= x <= 25`, `20 <= y <= 30` | Robotun basladigi ve bataryayi aldigi bolge |
| Charging docks | `0 <= x <= 25`, `10 <= y <= 20` | Gecislerde dikkat edilmesi gereken dock bolgesi |
| Main transit aisle | `25 <= x <= 50`, `10 <= y <= 30` | Ana ulasim koridoru |
| Drop-off | `x = 45`, `y = 10` | Batarya yerlestirme noktasi |

## Robot Hareket Rotasi

Robot dogrudan hedefe gitmez. Engel ve bolme yapisini dikkate alan waypoint rotasini takip eder.

Kodda kullanilan rota:

| Waypoint | Koordinat [m] | Aciklama |
|---|---:|---|
| P0 | `(5, 25)` | Baslangic / battery loading |
| P1 | `(5, 21)` | Warehouse cikisina yaklasma |
| P2 | `(5, 19)` | Gate-1 uzerinden charging docks tarafina gecis |
| P3 | `(12, 18)` | Sol bolgede guvenli gecis |
| P4 | `(24, 18)` | Main aisle giris kapisina yaklasma |
| P5 | `(28, 18)` | Gate-2 uzerinden ana koridora giris |
| P6 | `(40, 18)` | Ana route uzerinde ilerleme |
| P7 | `(45, 10)` | Drop-off hedefi |

Not: PDF'teki rota listesinde P2 `(12,21)` olarak verilmisti. Engel sisirme uygulandiginda `(12,21) -> (24,18)` gecisi B1 bolmesiyle cakistigi icin kodda Gate-1'den gercekten gececek sekilde `(5,19)` ve `(12,18)` ara noktalari kullanildi. Bu tercih, carpismasiz rota elde etmek icin yapildi.

Hareket parametreleri:

| Parametre | Deger |
|---|---:|
| Zaman adimi `dt` | `0.5 s` |
| Nominal hiz | `1.0 m/s` |
| Waypoint toleransi | `0.30 m` |
| Maksimum simülasyon adimi | `150` |
| Toplam simülasyon suresi | `74.5 s` |

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
| B1 | `y=20`, `0 <= x <= 25` | Warehouse ile charging docks arasindaki bolme |
| B2 | `x=25`, `12 <= y <= 30` | Sol bolgeler ile main transit aisle arasindaki bolme |
| B3 | `y=10`, `0 <= x <= 40` | Alt bolgeyi ayiran yatay sinir |
| B4 | `x=40`, `0 <= y <= 10` | Drop-off tarafindaki dikey sinir |

Kodda kapilar/acikliklar nedeniyle B1 ve B2 parcalara ayrilmistir.

Sisirilmis yasak dikdortgenler:

| Engel | Dikdortgen `(xmin, ymin, xmax, ymax)` | Neden |
|---|---|---|
| B1-left | `(0.0, 19.3, 4.0, 20.7)` | Gate-1 solundaki B1 parcasi |
| B1-right | `(6.0, 19.3, 25.0, 20.7)` | Gate-1 sagindaki B1 parcasi |
| B2-lower | `(24.3, 12.0, 25.7, 16.6)` | Gate-2 alt parcasi |
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

Alan koselerine yakin dengeli yerlesimdir. Bu geometri ana senaryo olarak kullanilir.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 1.0 | 1.0 |
| A2 | 49.0 | 1.0 |
| A3 | 49.0 | 29.0 |
| A4 | 1.0 | 29.0 |

### G2 Task Oriented

Robotun gorev rotasina ve drop-off alanina daha yakin anchorlar icerir. Pratikte bu geometri bazi rota bolumlerinde referans veya diger anchorlarin NLOS olmasina daha yatkindir.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 3.0 | 27.5 |
| A2 | 24.0 | 27.5 |
| A3 | 28.0 | 15.5 |
| A4 | 48.0 | 11.0 |

### G3 Poor Same Wall

Kotu geometri ornegidir. Anchorlar neredeyse ayni alt duvar hattindadir. Bu dizilim TDOA icin y yonundeki bilgiyi zayiflatir.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 2.0 | 2.0 |
| A2 | 17.0 | 2.0 |
| A3 | 33.0 | 2.0 |
| A4 | 48.0 | 2.0 |

Son calistirma sonucu:

| Geometri | Ortalama RMSE [m] | RMSE std [m] | Ortalama kosul sayisi |
|---|---:|---:|---:|
| G1 corner coverage | 0.764 | 0.081 | 2.828 |
| G2 task oriented | 2.327 | 0.972 | 34.135 |
| G3 poor same wall | 14.227 | 0.017 | 42.405 |

Yorum: G1 en dengeli sonuc verir. G3'un hatasi cok yuksektir cunku sensorler ayni hatta toplandiginda TDOA hiperbol kesismeleri kotu kosullanir. G2 rota yakinligi acisindan mantikli gorunse de NLOS etkisi ve kosullama nedeniyle G1 kadar basarili degildir.

## Sensor Sayisi Analizi

Sensör sayisi 4'ten baslatilip 10'a kadar arttirildi. Maksimum sensör sayisi bu asama icin `10` secildi.

Bu secimin gerekcesi:

- 50x30 m gibi orta boy bir fabrika plani icin 4 anchor TDOA'nin teorik minimum pratik baslangicidir.
- 6-8 anchor genellikle kapsama ve dayaniklilik icin yeterli iyilesme saglar.
- 10 anchor sonrasi ek maliyet artar, fakat bu harita olceginde beklenen ek kazanc azalir.
- Daha fazla anchor, NLOS anchor sayisini da artirabilecegi icin performans her zaman monoton iyilesmek zorunda degildir.

Kullanilan 10 sensorluk maksimum havuz:

| Sira | x [m] | y [m] | Aciklama |
|---|---:|---:|---|
| 1 | 1.0 | 1.0 | Sol alt kose, referans |
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
| 4 | 0.759 |
| 5 | 0.757 |
| 6 | 0.716 |
| 7 | 0.738 |
| 8 | 0.732 |
| 9 | 0.750 |
| 10 | 0.740 |

Yorum: En iyi ortalama sonuc bu calistirmada 6 sensor civarinda goruldu. 7-10 sensor araliginda hata tekrar biraz artabiliyor; bunun nedeni yeni eklenen bazi anchorlarin belirli rota bolumlerinde NLOS olcumu uretmesi ve TDOA referans yapisinin dogrusal olmayan etkileridir. Bu bulgu raporda "sensör sayisi artisi her zaman tek basina yeterli degildir; geometri ve NLOS kosullari birlikte degerlendirilmelidir" seklinde yorumlanabilir.

## Revizyon Icin Kodda Bakilacak Yerler

Temel ortam parametreleri `Config` sinifindadir:

- `factory_size`: fabrika boyutu
- `dt`: zaman adimi
- `nominal_speed`: robot hizi
- `waypoint_tolerance`: waypoint'e ulasma toleransi
- `robot_radius`, `safety_margin`: engel sisirme parametreleri
- `sigma_toa_los`, `sigma_toa_nlos`: UWB olcum gurultuleri
- `sigma_accel`: EKF surec gurultusu
- `monte_carlo_runs`: analiz tekrar sayisi

Rota:

- `WAYPOINTS`

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

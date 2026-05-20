# Fabrika Ortamında Batarya Yerleştirme Robotunun UWB/TDOA ve EKF ile Takibi

Bu proje, iki boyutlu bir fabrika ortamında hareket eden batarya yerleştirme görevli mobil robotun UWB tabanlı TDOA ölçümleri kullanılarak takip edilmesini amaçlamaktadır. Robotun gerçek hareketi simülasyon ortamında üretilmekte, sabit konumlu UWB sensörlerinden gürültülü TDOA ölçümleri alınmakta, ilk konum tahmini LSE yöntemiyle yapılmakta ve takip işlemi EKF ile gerçekleştirilmektedir.

Projedeki ana inceleme konusu, sensör geometrisinin takip başarısına etkisidir. Bu nedenle özellikle 4 sensörlü üç farklı geometri ayrıntılı olarak incelenmiş, ayrıca sensör sayısı 4'ten 10'a kadar artırıldığında performansın nasıl değiştiği analiz edilmiştir.

## Projenin Amacı

Akıllı fabrikalarda mobil robotlar üretim hattı, depo, şarj alanı ve teslim noktaları arasında hareket eder. Bu robotların konumunun güvenilir biçimde takip edilmesi; görev güvenliği, rota doğrulama ve fabrika otomasyonu açısından önemlidir.

Bu projede ele alınan görev şu şekildedir:

1. Robot `Parking Docks` alanından çıkar.
2. `Battery Loading / Pickup` noktasına gider.
3. Pickup noktasında 5 saniye bekleyerek batarya yükleme işlemini temsil eder.
4. Gate-2 açıklığından ana koridora çıkar.
5. Ana koridor üzerinden en az yön değişimiyle `Drop-off` noktasına ilerler.
6. Drop-off noktasına ulaştığında simülasyon sonlandırılır.

## Kullanılan Yöntemlerin Özeti

| Bileşen | Kullanılan yöntem | Açıklama |
|---|---|---|
| Çalışma ortamı | 2B fabrika geometrisi | Duvarlar, kapılar, bölgeler ve şişirilmiş engeller tanımlanmıştır. |
| Robot hareketi | Sabit hızlı görev fazları | Robot belirlenen görev noktaları arasında 1 m/s hızla ilerler. |
| Ölçüm sistemi | UWB/TDOA | Sensörler arası varış zamanı farkından menzil farkı ölçümü üretilir. |
| Gürültü modeli | LOS/NLOS Gauss gürültüsü | Engel kesişimi olan bağlantılarda daha yüksek gürültü kullanılır. |
| İlk konum tahmini | LSE / Gauss-Newton | EKF başlangıç durumu ilk TDOA ölçümlerinden tahmin edilir. |
| Takip algoritması | EKF | Doğrusal olmayan TDOA ölçüm modeli Jacobian ile kullanılır. |
| Performans ölçütü | RMSE | Gerçek ve tahmin edilen konumlar arasındaki hata değerlendirilir. |

## Dosya Yapısı

| Dosya / Klasör | Açıklama |
|---|---|
| `agv_tdoa_ekf.py` | Ana simülasyon, TDOA ölçüm üretimi, LSE ilklendirme, EKF takip ve sensör analiz kodu. |
| `factory_geometry_visual.py` | Yalnızca fabrika geometrisini gösteren açıklamalı görsel üretim kodu. |
| `generate_analysis_figures.py` | 4 sensörlü A1 önce/sonra analizi ve 4-10 sensör artış analizi için ayrıntılı grafik üretim kodu. |
| `requirements.txt` | Gerekli Python paketleri. |
| `outputs/` | Simülasyon ve analiz sonucu oluşan grafikler ve CSV dosyaları. |

## Çalıştırma

Gerekli paketleri kurmak için:

```powershell
pip install -r requirements.txt
```

Ana takip simülasyonunu çalıştırmak için:

```powershell
C:\ANACONDA\python.exe agv_tdoa_ekf.py
```

Yalnızca fabrika geometrisini çizmek için:

```powershell
C:\ANACONDA\python.exe factory_geometry_visual.py
```

Ayrıntılı 4 sensör ve 4-10 sensör grafiklerini üretmek için:

```powershell
C:\ANACONDA\python.exe generate_analysis_figures.py
```

## Fabrika Ortamı ve Koordinat Sistemi

Fabrika ortamı iki boyutlu Kartezyen koordinat sistemiyle modellenmiştir. Koordinatlar metre cinsindendir.

| Özellik | Değer |
|---|---:|
| Harita genişliği | 50 m |
| Harita yüksekliği | 30 m |
| Orijin | Sol-alt köşe, `(0, 0)` |
| x ekseni | Sağa doğru artar |
| y ekseni | Yukarı doğru artar |

Alan sınırları:

```text
0 <= x <= 50
0 <= y <= 30
```

## Fabrika Bölgeleri

Fabrika, robot görevinin anlaşılması için bölgelere ayrılmıştır. Bu bölgeler takip algoritmasına doğrudan ölçüm olarak girmez; ortamın ve görevin açıklanmasını sağlar.

| Bölge | Koordinat aralığı | Görevdeki rolü |
|---|---|---|
| `Parking Docks` | `0 <= x <= 25`, `20 <= y <= 30` | Robotun göreve başladığı park/dock bölgesi. |
| `Battery Loading / Pickup` | `0 <= x <= 25`, `10 <= y <= 20` | Robotun bataryayı teslim aldığı bölge. |
| `Main Transit Aisle` | `25 <= x <= 50`, `10 <= y <= 30` | Robotun drop-off noktasına ilerlediği ana koridor. |
| `Lower Service Area` | `0 <= x <= 50`, `0 <= y <= 10` | Ana görev rotasında kullanılmayan alt servis bölgesi. |
| `Drop-off` | `(45, 10)` | Bataryanın teslim edildiği hedef nokta. |

## Görev Noktaları ve Robot Hareketi

Robotun hareketi görev fazları üzerinden tanımlanmıştır. Simülasyon, robotun `Parking Docks` noktasından çıkmasıyla başlar ve drop-off noktasına ulaştığında sona erer.

| Görev noktası | Koordinat | Açıklama |
|---|---:|---|
| M0 | `(5, 25)` | Başlangıç noktası, Parking Docks. |
| M1 | `(5, 15)` | Pickup noktası, batarya teslim alma. |
| M2 | `(25, 18)` | Gate-2 açıklığı ve ana koridora giriş. |
| M3 | `(45, 10)` | Drop-off hedef noktası. |

Hareket fazları:

| Faz | Hareket | Hız | Açıklama |
|---|---|---:|---|
| 1 | `(5,25) -> (5,15)` | 1.0 m/s | Robot Gate-1 üzerinden dikey olarak pickup noktasına iner. |
| 2 | `(5,15)` üzerinde bekleme | 0.0 m/s | Robot 5 saniye durur ve batarya yükleme işlemi temsil edilir. |
| 3 | `(5,15) -> (25,18)` | 1.0 m/s | Robot Gate-2 açıklığından ana koridora çıkar. |
| 4 | `(25,18) -> (45,10)` | 1.0 m/s | Robot drop-off noktasına en az yön değişimiyle ilerler. |

Hareket parametreleri:

| Parametre | Değer |
|---|---:|
| Zaman adımı `dt` | 0.5 s |
| Nominal hız | 1.0 m/s |
| Pickup bekleme süresi | 5.0 s |
| Toplam simülasyon süresi | 57.5 s |
| Simülasyon bitiş koşulu | Drop-off noktasına ulaşma |

## Duvarlar, Kapılar ve Engeller

Fabrika içindeki duvarlar ve bölmeler robot için fiziksel engel kabul edilmiştir. Robot noktasal olarak modellenmiştir; ancak gerçek robot boyutu ve güvenlik mesafesi dikkate alınarak duvarlar şişirilmiş yasak bölgeler haline getirilmiştir.

| Parametre | Değer |
|---|---:|
| Robot yarıçapı | 0.35 m |
| Güvenlik mesafesi | 0.25 m |
| Toplam güvenlik payı | 0.60 m |
| Ham duvar kalınlığı | 0.20 m |
| Şişirilmiş yarı kalınlık | 0.70 m |

Ham duvar ve bölme segmentleri:

| Kod | Geometri | Açıklama |
|---|---|---|
| B0 | Dış sınır: `x=0`, `x=50`, `y=0`, `y=30` | Robot harita dışına çıkamaz. |
| B1 | `y=20`, `0 <= x <= 25` | Parking Docks ile Battery Loading / Pickup arasındaki yatay bölme. |
| B2 | `x=25`, `10 <= y <= 30` | Sol bölgeler ile ana koridor arasındaki dikey bölme. |
| B3 | `y=10`, `0 <= x <= 40` | Alt servis bölgesini ayıran yatay duvar. |
| B4 | `x=40`, `0 <= y <= 10` | Drop-off tarafındaki dikey alt bölme. |

Kapılar ve açıklıklar:

| Kapı / açıklık | Koordinat tanımı | Açıklama |
|---|---|---|
| Gate-1 | B1 üzerinde `x=4..6` aralığı | Robotun Parking Docks bölgesinden Battery Loading / Pickup bölgesine geçmesini sağlar. |
| Gate-2 | B2 üzerinde `16.6 <= y <= 19.0` aralığı | Robotun sol bölgeden ana koridora geçmesini sağlar. |
| Drop-off yaklaşımı | B3 yalnızca `x <= 40` için engeldir | Robotun `x > 40` tarafından drop-off noktasına yaklaşmasına izin verir. |

Şişirilmiş yasak bölgeler:

| Engel | Dikdörtgen `(xmin, ymin, xmax, ymax)` | Açıklama |
|---|---|---|
| B1-left | `(0.0, 19.3, 4.0, 20.7)` | Gate-1 solundaki B1 parçası. |
| B1-right | `(6.0, 19.3, 25.0, 20.7)` | Gate-1 sağındaki B1 parçası. |
| B2-lower | `(24.3, 10.0, 25.7, 16.6)` | Gate-2 alt parçası. B2-lower ile B3 arasındaki gereksiz boşluk kaldırılmıştır. |
| B2-upper | `(24.3, 19.0, 25.7, 30.0)` | Gate-2 üst parçası. |
| B3 | `(0.0, 9.3, 40.0, 10.7)` | Alt yatay duvarın şişirilmiş hali. |
| B4 | `(39.3, 0.0, 40.7, 10.0)` | Drop-off tarafındaki dikey duvarın şişirilmiş hali. |

## UWB/TDOA Ölçüm Modeli

Robotun konumu doğrudan ölçülmez. Bunun yerine fabrika içine yerleştirilmiş UWB anchor sensörleri robot sinyalinin varış zamanını algılar. Bu projede mutlak varış zamanı yerine TDOA yöntemi kullanılmıştır.

TDOA yönteminde bir sensör referans seçilir. Bu projede referans sensör `A1`'dir. Her ölçüm, diğer sensör ile referans sensör arasındaki menzil farkı olarak yazılır:

```text
z_i = ||p - A_i|| - ||p - A_ref|| + v_i
```

Burada:

| Sembol | Anlam |
|---|---|
| `p` | Robotun iki boyutlu konumu, `[x, y]^T`. |
| `A_i` | i'inci UWB sensörünün konumu. |
| `A_ref` | Referans sensör olan A1'in konumu. |
| `v_i` | TDOA menzil farkı ölçüm gürültüsü. |

`M` adet sensör kullanıldığında ölçüm boyutu `M-1` olur. Örneğin 4 sensörlü durumda 1 referans sensör ve 3 menzil farkı ölçümü vardır.

## Gürültü ve NLOS Modeli

Fabrika ortamında duvarlar ve bölmeler UWB sinyalini bozabilir. Robot ile sensör arasındaki doğru parçası şişirilmiş bir engelden geçerse bu bağlantı NLOS kabul edilir.

| Durum | TOA standart sapması | Menzil karşılığı | Anlam |
|---|---:|---:|---|
| LOS | `0.25 / c` s | 0.25 m | Robot ve sensör arasında engel yoktur. |
| NLOS | `1.50 / c` s | 1.50 m | Sinyal engelden etkilenmiştir. |

TDOA kovaryansı şu şekilde hesaplanır:

```text
Var(z_i) = c^2 * (sigma_i^2 + sigma_ref^2)
```

Referans sensör A1'in NLOS olması önemlidir; çünkü referans sensördeki hata tüm TDOA fark ölçümlerini etkiler.

## LSE ile Başlangıç Konumu Tahmini

EKF algoritmasının başlatılabilmesi için ilk durum vektörüne ihtiyaç vardır. Bu projede ilk konum LSE yöntemiyle tahmin edilmiştir.

Uygulanan işlem:

1. İlk 8 zaman adımındaki TDOA ölçümleri alınır.
2. Her zaman adımı için Gauss-Newton LSE ile robot konumu `[x, y]` tahmin edilir.
3. İlk konum, ilk LSE konum tahmininden alınır.
4. İlk hız, ilk 8 LSE konumuna doğrusal eğri uydurularak hesaplanır.
5. İlk kovaryans matrisi, LSE Jacobian yaklaşımından elde edilir.

LSE problemi:

```text
min_p || z - h(p) ||_R
```

## EKF Takip Modeli

EKF durum vektörü:

```text
X = [x, y, vx, vy]^T
```

Hareket modeli sabit hızlıdır:

```text
x_k  = x_{k-1}  + dt * vx_{k-1}
y_k  = y_{k-1}  + dt * vy_{k-1}
vx_k = vx_{k-1}
vy_k = vy_{k-1}
```

Süreç gürültüsü robotun küçük hız sapmalarını ve modelleme belirsizliklerini temsil eder:

| Parametre | Değer |
|---|---:|
| `sigma_accel` | 0.35 m/s² |

EKF tahmin adımı:

```text
X_pred = F X_prev
P_pred = F P_prev F^T + Q
```

EKF güncelleme adımı:

```text
y = z - h(X_pred)
S = H P_pred H^T + R
K = P_pred H^T S^-1
X = X_pred + K y
P = (I - K H) P_pred (I - K H)^T + K R K^T
```

TDOA ölçüm fonksiyonu doğrusal olmadığı için EKF'de Jacobian kullanılır:

```text
dh_i/dp = (p - A_i) / ||p - A_i|| - (p - A_ref) / ||p - A_ref||
```

## Dört Sensörlü Geometriler

4 sensörlü durum bu projenin ana inceleme alanıdır. Her geometride `A1` referans sensördür. Güncel durumda A1, B3 duvarının sol ucu olan `(0, 10)` konumuna alınmıştır.

### G1: Corner Coverage

Bu geometri, sensörleri alanın farklı köşelerine yakın yerleştirerek dengeli kapsama sağlamayı amaçlar.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 0.0 | 10.0 |
| A2 | 49.0 | 1.0 |
| A3 | 49.0 | 29.0 |
| A4 | 1.0 | 29.0 |

### G2: Task Oriented

Bu geometri, robotun görev rotası ve drop-off çevresine daha yakın sensörler kullanır.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 0.0 | 10.0 |
| A2 | 24.0 | 27.5 |
| A3 | 28.0 | 15.5 |
| A4 | 48.0 | 11.0 |

### G3: Poor Same Wall

Bu geometri kötü koşulları temsil eder. A1 B3 sol ucunda sabit tutulurken diğer sensörler alt hatta yakın seçilmiştir.

| Anchor | x [m] | y [m] |
|---|---:|---:|
| A1 | 0.0 | 10.0 |
| A2 | 17.0 | 2.0 |
| A3 | 33.0 | 2.0 |
| A4 | 48.0 | 2.0 |

Ana simülasyon çıktısına göre 4 sensörlü geometri sonuçları:

| Geometri | Ortalama RMSE [m] | RMSE std [m] | Ortalama koşul sayısı |
|---|---:|---:|---:|
| G1 Corner Coverage | 0.832 | 0.077 | 4.796 |
| G2 Task Oriented | 1.387 | 0.286 | 21.713 |
| G3 Poor Same Wall | 2.829 | 2.038 | 42.051 |

Bu sonuçlara göre G1 en düşük ortalama RMSE değerini vermektedir. G2, görev rotasına yakın sensörler kullandığı için bazı durumlarda iyi sonuç üretse de koşul sayısı G1'e göre daha yüksektir. G3 ise geometrik olarak daha zayıftır; hata ortalaması ve değişkenliği daha fazladır.

## A1 Sensörü Önce / Sonra Karşılaştırması

Son revizyonda A1 sensörü B3 duvarının sol ucuna alınmıştır.

| Durum | A1 konumu |
|---|---:|
| Revizyon öncesi | `(1, 1)` |
| Revizyon sonrası | `(0, 10)` |

Bu karşılaştırmada iki farklı sonuç türü vardır:

| Sonuç türü | Açıklama |
|---|---|
| Ana takip RMSE | Tek bir sabit rastgele tohumla çalıştırılan ana senaryo sonucudur. |
| Ortalama RMSE | Monte Carlo tekrarlarının ortalamasıdır ve geometri karşılaştırması için daha güvenilir kabul edilir. |

| Metrik | A1 önce `(1,1)` | A1 sonra `(0,10)` | Yorum |
|---|---:|---:|---|
| Ana takip RMSE | 0.692 | 0.675 | Tekil ana senaryoda iyileşme vardır. |
| G1 4 sensör ortalama RMSE | 0.814 | 0.832 | G1 için küçük bir kötüleşme vardır. |
| G2 4 sensör ortalama RMSE | 1.860 | 1.387 | G2 belirgin biçimde iyileşmiştir. |
| G3 4 sensör ortalama RMSE | 14.607 | 2.829 | G3 çok ciddi biçimde iyileşmiştir. |
| 8 sensör ortalama RMSE | 0.737 | 0.743 | Değişim çok küçüktür. |
| 10 sensör ortalama RMSE | 0.744 | 0.744 | Pratikte aynı kalmıştır. |

Bu nedenle A1'in B3 sol ucuna alınması her durumda mutlak bir iyileşme sağlamaz. Ana takip senaryosunda, G2'de ve G3'te iyileşme vardır; ancak dengeli G1 geometrisinde küçük bir performans kaybı oluşmaktadır. Rapor yorumunda bu fark açıkça belirtilmelidir.

## Sensör Sayısı Analizi

Sensör sayısı 4'ten 10'a kadar artırılmıştır. Maksimum sensör sayısı bu aşama için 10 seçilmiştir. Bunun nedeni 50 m x 30 m büyüklüğündeki fabrika alanında 10 sensörün hem kapsama açısından yeterli bir üst sınır olması hem de daha fazla sensörün maliyet ve karmaşıklık açısından anlamlı bir ilk analiz sınırını aşmasıdır.

Kullanılan 10 sensörlük maksimum havuz:

| Sıra | x [m] | y [m] | Açıklama |
|---:|---:|---:|---|
| 1 | 0.0 | 10.0 | A1, B3 duvarının sol ucu ve referans sensör. |
| 2 | 49.0 | 1.0 | Sağ alt köşe. |
| 3 | 49.0 | 29.0 | Sağ üst köşe. |
| 4 | 1.0 | 29.0 | Sol üst köşe. |
| 5 | 25.0 | 29.0 | Üst orta. |
| 6 | 49.0 | 15.0 | Sağ orta. |
| 7 | 1.0 | 15.0 | Sol orta. |
| 8 | 25.0 | 1.0 | Alt orta. |
| 9 | 25.0 | 18.0 | Gate-2 / ana koridor yakınında kolon anchor. |
| 10 | 45.0 | 10.0 | Drop-off yakın anchor. |

Ana simülasyon çıktısına göre sensör sayısı analizi:

| Sensör sayısı | Ortalama RMSE [m] |
|---:|---:|
| 4 | 0.843 |
| 5 | 0.796 |
| 6 | 0.772 |
| 7 | 0.795 |
| 8 | 0.743 |
| 9 | 0.759 |
| 10 | 0.744 |

Bu sonuçlara göre en iyi ortalama RMSE değeri 8 sensörde elde edilmiştir. Ancak performans sensör sayısıyla monoton olarak iyileşmemektedir. Bunun temel nedeni, eklenen her sensörün geometriyi her zaman iyileştirmemesi ve bazı sensörlerin NLOS ölçüm üretme olasılığını artırmasıdır.

## Ana Çıktılar

| Çıktı | Açıklama |
|---|---|
| `outputs/factory_geometry_only.png` | Sensör ve takip sonucu olmadan yalnızca fabrika geometrisini gösterir. |
| `outputs/battery_robot_tracking_map.png` | Robotun gerçek yolu, EKF tahmini, sensörler ve NLOS anlarını gösterir. |
| `outputs/tracking_time_series.png` | x/y konumları ve zamana bağlı konum hatasını gösterir. |
| `outputs/geometry_and_sensor_count_analysis.png` | 4 sensör geometrileri ve 4-10 sensör sayısı analizini özetler. |
| `outputs/tracking_results.csv` | Zaman, gerçek konum, tahmin konumu, hata ve NLOS sensör sayısını içerir. |
| `outputs/main_metrics.csv` | Ana senaryonun RMSE, ortalama hata ve maksimum hata metriklerini içerir. |
| `outputs/four_sensor_geometry_analysis.csv` | 4 sensörlü G1/G2/G3 geometrilerinin sayısal sonuçlarını içerir. |
| `outputs/sensor_count_analysis.csv` | 4-10 sensör analizinin sayısal sonuçlarını içerir. |

## Ayrıntılı Grafik Dosyaları

4 sensörlü geometri ana inceleme alanı olduğu için A1 revizyonu öncesi ve sonrası ayrı klasörlerde gösterilmiştir.

| Klasör / Dosya | Açıklama |
|---|---|
| `outputs/analysis_4_sensor_a1_before/` | A1 eski konumdayken G1, G2 ve G3 için ayrı grafikler. |
| `outputs/analysis_4_sensor_a1_after/` | A1 `(0,10)` konumundayken G1, G2 ve G3 için ayrı grafikler. |
| `outputs/analysis_4_sensor_a1_before_after_summary.png` | A1 önce/sonra karşılaştırmasını tek grafikte gösterir. |
| `outputs/analysis_sensor_count_4_to_10/` | 4'ten 10'a kadar her sensör sayısı için ayrı grafikler. |

Bu klasörlerdeki her `map_and_error.png` dosyası iki bilgiyi birlikte verir:

1. Sol tarafta sensörlerin haritadaki konumları, gerçek robot yolu ve EKF tahmini.
2. Sağ tarafta zamanla değişen konum hatası.

Not: `generate_analysis_figures.py`, ana simülasyon betiğinden bağımsız sabit rastgele tohumlar kullanarak yeniden Monte Carlo hesabı yapar. Bu nedenle bu klasörlerdeki `summary.csv` değerleri ile ana README tablolarındaki değerler arasında küçük sayısal farklar olabilir. Karşılaştırma yapılırken aynı analiz ailesindeki grafik ve CSV dosyaları birlikte değerlendirilmelidir.

## Kod Üzerinde Değişiklik Yapılacak Yerler

| Değiştirilecek unsur | Kod içindeki yer |
|---|---|
| Fabrika boyutu | `Config.factory_size` |
| Zaman adımı | `Config.dt` |
| Robot hızı | `Config.nominal_speed` |
| Pickup bekleme süresi | `Config.pickup_wait_time` |
| Robot yarıçapı ve güvenlik payı | `Config.robot_radius`, `Config.safety_margin` |
| LOS/NLOS gürültüsü | `Config.sigma_toa_los`, `Config.sigma_toa_nlos` |
| EKF süreç gürültüsü | `Config.sigma_accel` |
| Monte Carlo tekrar sayısı | `Config.monte_carlo_runs` |
| Görev noktaları | `MISSION_POINTS` |
| 4 sensör geometrileri | `SENSOR_GEOMETRIES_4` |
| 4-10 sensör havuzu | `SENSOR_POOL_MAX_10` |
| Duvar, kapı ve engel modeli | `inflated_obstacles(cfg)` |

## Rapor İçin Önerilen Bölümler

1. Problem tanımı ve motivasyon
2. Fabrika ortamının geometrik modeli
3. Robot görev ve hareket modeli
4. UWB/TDOA ölçüm modeli
5. LOS/NLOS gürültü modeli
6. LSE ile başlangıç tahmini
7. EKF takip algoritması
8. 4 sensörlü geometri analizi
9. A1 sensörü önce/sonra karşılaştırması
10. 4-10 sensör sayısı analizi
11. Sonuçların yorumlanması
12. Gelecek geliştirme önerileri

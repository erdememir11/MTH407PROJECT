# Akıllı Fabrika Ortamında UWB/TDOA Tabanlı Robot Takibi

Bu proje, kapalı bir fabrika ortamında görev yapan batarya taşıma robotunun konumunun UWB sensörleriyle takip edilmesini inceler. Robot, `Parking Docks` bölgesinden çıkar, `Battery Loading / Pickup` noktasında bataryayı alır, `Gate-2` açıklığından ana koridora geçer ve `Drop-off` noktasına ulaşınca görevini tamamlar.

Projenin ana amacı yalnızca bir robot rotası çizmek değildir. Amaç; fabrika geometrisini, robot hareketini, UWB/TDOA ölçüm modelini, ölçüm gürültüsünü, LSE başlangıç tahminini, EKF takip algoritmasını ve sensör geometrisi etkisini tek bir simülasyon yapısı içinde incelemektir.

## 1. Proje Dosya Yapısı

Word şablonuna göre proje kök dizininde yalnızca Python dosyaları ve README dosyası bulunacak şekilde düzenlenmiştir.

| Dosya | Görevi |
|---|---|
| `project_model.py` | Ortam, robot hareketi, TDOA ölçümü, LSE, EKF, metrik ve ortak çizim fonksiyonlarını içerir. |
| `generate_01_environment.py` | Fabrika ortamı ve beklenen robot hareketi için görselleri üretir. |
| `analysis_pipeline.py` | Gürültü senaryoları için ortak analiz ve grafik üretim fonksiyonlarını içerir. |
| `run_02_baseline_constant_noise.py` | Tüm fabrika boyunca yalnızca LOS gürültüsü varsayımıyla analiz yapar. |
| `run_03_realistic_los_nlos_noise.py` | Gerçekçi LOS/NLOS gürültü modeliyle analiz yapar. |
| `run_all_outputs.py` | Üç çıktı klasörünün tamamını tek komutla yeniden üretir. |
| `README.md` | Projenin amacı, modeli, dosya yapısı ve çıktıların açıklamasıdır. |

## 2. Çalıştırma

Tüm çıktıları üretmek için:

```powershell
C:\ANACONDA\python.exe run_all_outputs.py
```

Yalnızca ortam çıktıları için:

```powershell
C:\ANACONDA\python.exe generate_01_environment.py
```

Yalnızca sabit LOS gürültü senaryosu için:

```powershell
C:\ANACONDA\python.exe run_02_baseline_constant_noise.py
```

Yalnızca gerçekçi LOS/NLOS gürültü senaryosu için:

```powershell
C:\ANACONDA\python.exe run_03_realistic_los_nlos_noise.py
```

## 3. Çıktı Klasörleri

Şablona uygun olarak tüm çıktılar `outputs/` altında üç ana klasöre ayrılmıştır.

| Klasör | İçerik |
|---|---|
| `outputs/01_environment/` | Simülasyon ortamı ve robot hareketiyle ilgili görseller. |
| `outputs/02_baseline_constant_noise/` | Tüm fabrika boyunca LOS gürültüsü varsayımıyla üretilen takip ve analiz çıktıları. |
| `outputs/03_realistic_los_nlos_noise/` | LOS ve NLOS çeşitliliği içeren gerçekçi gürültü modeliyle üretilen takip ve analiz çıktıları. |

## 4. Simülasyon Ortamı

Fabrika ortamı iki boyutlu Kartezyen koordinat sisteminde modellenmiştir.

| Özellik | Değer |
|---|---:|
| Genişlik | 50 m |
| Yükseklik | 30 m |
| Orijin | Sol-alt köşe, `(0, 0)` |
| x ekseni | Sağa doğru artar |
| y ekseni | Yukarı doğru artar |

Fabrika bölgeleri:

| Bölge | Koordinat aralığı | Açıklama |
|---|---|---|
| `Parking Docks` | `0 <= x <= 25`, `20 <= y <= 30` | Robotun göreve başladığı park alanı. |
| `Battery Loading / Pickup` | `0 <= x <= 25`, `10 <= y <= 20` | Robotun bataryayı aldığı bölge. |
| `Main Transit Aisle` | `25 <= x <= 50`, `10 <= y <= 30` | Robotun drop-off noktasına ilerlediği ana koridor. |
| `Lower Service Area` | `0 <= x <= 50`, `0 <= y <= 10` | Ana görev rotasının doğrudan kullanmadığı alt servis bölgesi. |

Duvarlar ve açıklıklar:

| Kod | Geometri | Açıklama |
|---|---|---|
| B1 | `y=20`, `0 <= x <= 25` | Parking Docks ile Battery Loading / Pickup arasındaki bölme. Gate-1 açıklığı vardır. |
| B2 | `x=25`, `10 <= y <= 30` | Sol bölgeler ile ana koridor arasındaki bölme. Gate-2 açıklığı vardır. |
| B3 | `y=10`, `0 <= x <= 40` | Alt servis bölgesini ayıran yatay duvar. |
| B4 | `x=40`, `0 <= y <= 10` | Drop-off tarafındaki dikey alt duvar. |

Engeller robot yarıçapı ve güvenlik mesafesi dikkate alınarak şişirilmiştir.

| Parametre | Değer |
|---|---:|
| Robot yarıçapı | 0.35 m |
| Güvenlik mesafesi | 0.25 m |
| Toplam güvenlik payı | 0.60 m |
| Ham duvar kalınlığı | 0.20 m |
| Şişirilmiş yarı kalınlık | 0.70 m |

## 5. Robot Hareket Modeli

Robot hareketi görev fazlarıyla tanımlanmıştır.

| Görev noktası | Koordinat | Açıklama |
|---|---:|---|
| M0 | `(5, 25)` | Parking Docks başlangıç noktası. |
| M1 | `(5, 15)` | Pickup noktası. Robot burada 5 saniye bekler. |
| M2 | `(25, 18)` | Gate-2 açıklığı ve ana koridora giriş noktası. |
| M3 | `(45, 10)` | Drop-off hedef noktası. |

Hareket fazları:

| Faz | Hareket | Hız | Açıklama |
|---|---|---:|---|
| 1 | `(5,25) -> (5,15)` | 1.0 m/s | Robot Gate-1 üzerinden dikey olarak pickup noktasına iner. |
| 2 | `(5,15)` üzerinde bekleme | 0.0 m/s | 5 saniyelik batarya yükleme işlemi temsil edilir. |
| 3 | `(5,15) -> (25,18)` | 1.0 m/s | Robot Gate-2 açıklığından ana koridora çıkar. |
| 4 | `(25,18) -> (45,10)` | 1.0 m/s | Robot drop-off noktasına en az yön değişimiyle ilerler. |

## 6. Ölçüm Modeli

Projede UWB tabanlı TDOA ölçüm modeli kullanılmaktadır. TDOA, sensörlere ulaşan sinyalin varış zamanı farkından menzil farkı üretir.

Ölçüm modeli:

```text
z_i = ||p - A_i|| - ||p - A_ref|| + v_i
```

Burada:

| Sembol | Açıklama |
|---|---|
| `p` | Robot konumu, `[x, y]^T`. |
| `A_i` | i'inci UWB sensörü. |
| `A_ref` | Referans sensör, yani A1. |
| `v_i` | Ölçüm gürültüsü. |

4 sensör kullanıldığında bir sensör referans alınır ve 3 adet TDOA menzil farkı ölçümü elde edilir. Genel olarak `M` sensör için ölçüm boyutu `M-1` olur.

## 7. Gürültü Senaryoları

Projede iki ayrı gürültü senaryosu incelenmektedir.

### 7.1. Baseline Constant Noise

Bu senaryoda tüm fabrika boyunca sensör-robot bağlantılarının LOS olduğu varsayılır. Yani her ölçüm düşük gürültülüdür.

| Durum | Menzil karşılığı |
|---|---:|
| LOS | 0.25 m |

Bu senaryo, çevresel engellerin ölçüme etkisini kapatıp yalnızca sensör geometrisinin temel etkisini görmek için kullanılır.

### 7.2. Realistic LOS/NLOS Noise

Bu senaryoda robot ile sensör arasındaki doğru parçası şişirilmiş bir engelden geçerse ölçüm NLOS kabul edilir. NLOS ölçümlerde gürültü daha yüksektir.

| Durum | Menzil karşılığı |
|---|---:|
| LOS | 0.25 m |
| NLOS | 1.50 m |

Bu senaryo gerçek fabrika ortamına daha yakındır; çünkü duvarlar, bölmeler ve makineler UWB sinyalini bozabilir.

## 8. Takip Algoritması

Takip süreci iki aşamadan oluşur:

1. İlk konum LSE yöntemiyle tahmin edilir.
2. Zaman içindeki konum ve hız EKF ile takip edilir.

EKF durum vektörü:

```text
X = [x, y, vx, vy]^T
```

Sabit hızlı hareket modeli:

```text
x_k  = x_{k-1}  + dt * vx_{k-1}
y_k  = y_{k-1}  + dt * vy_{k-1}
vx_k = vx_{k-1}
vy_k = vy_{k-1}
```

Performans ölçütü olarak RMSE kullanılmıştır:

```text
RMSE = sqrt(mean((x_est - x_true)^2 + (y_est - y_true)^2))
```

## 9. Dört Sensörlü Geometriler

4 sensörlü geometri projenin ana inceleme alanıdır. Şablonda verilen altı farklı sensör yerleşimi kullanılmıştır.

| Geometri | A1 | A2 | A3 | A4 | Test edilen fikir |
|---|---|---|---|---|---|
| G1 Balanced Corner Coverage | `(1,29)` | `(49,29)` | `(49,1)` | `(1,1)` | En dengeli köşe kapsaması. |
| G2 Task-Oriented Route Coverage | `(1,29)` | `(8,16)` | `(27,18)` | `(46,12)` | Sensörleri robot rotasına yakın koymak işe yarıyor mu? |
| G3 Poor Bottom Cluster | `(1,29)` | `(16,2)` | `(32,2)` | `(48,2)` | Alt hatta yığılmış kötü geometri. |
| G4 B3 Wall-Aware Coverage | `(1,29)` | `(49,29)` | `(1,11.2)` | `(39,11.2)` | B3 duvar uçlarını izlemek faydalı mı? |
| G5 Left-Bottom + B3-Right | `(1,29)` | `(49,29)` | `(1,1)` | `(39,11.2)` | Sol alt + B3 sağ asimetrisi. |
| G6 Right-Bottom + B3-Left | `(1,29)` | `(49,29)` | `(49,1)` | `(1,11.2)` | Sağ alt + B3 sol asimetrisi. |

Her gürültü senaryosu için bu 6 geometri ayrı ayrı çizdirilir. Her görselde gerçek robot yolu, EKF tahmin yolu, sensör konumları ve hata-zaman grafiği bulunur.

## 10. Sensör Sayısı Analizi

4 sensörlü analizden sonra sensör sayısı 4'ten 10'a kadar artırılmıştır. Bu analiz, sensör sayısı arttıkça sistem başarısının nasıl değiştiğini görmek için yapılır.

Kullanılan maksimum 10 sensörlük havuz:

| Sıra | Koordinat | Açıklama |
|---:|---:|---|
| 1 | `(1,29)` | Referans A1. |
| 2 | `(49,29)` | Sağ üst. |
| 3 | `(49,1)` | Sağ alt. |
| 4 | `(1,1)` | Sol alt. |
| 5 | `(8,16)` | Rota yakınında sol iç bölge. |
| 6 | `(27,18)` | Gate-2 ve ana koridor yakını. |
| 7 | `(46,12)` | Drop-off yakını. |
| 8 | `(1,11.2)` | B3 sol ucu yakını. |
| 9 | `(39,11.2)` | B3 sağ ucu yakını. |
| 10 | `(25,18)` | Ana koridor giriş noktası. |

## 11. Çıktıların İçeriği

### 11.1. `outputs/01_environment/`

| Dosya | Açıklama |
|---|---|
| `factory_environment_details.png` | Fabrikanın bölgelerini, duvarlarını, kapılarını ve şişirilmiş engellerini gösterir. |
| `expected_robot_motion.png` | Robotun beklenen görev hareketini fabrika haritası üzerinde gösterir. |

### 11.2. `outputs/02_baseline_constant_noise/`

| Çıktı türü | Açıklama |
|---|---|
| `01_...true_vs_estimate.png` - `06_...true_vs_estimate.png` | 4 sensörlü 6 geometri için gerçek yol ve tahmin yolu. |
| `four_sensor_error_time_comparison.png` | 6 geometrinin zamana bağlı hata karşılaştırması. |
| `four_sensor_rmse_comparison.png` | 6 geometrinin RMSE karşılaştırması. |
| `sensor_count_4_to_10_comparison.png` | Sensör sayısı 4'ten 10'a çıktığında sistem performansı. |
| `four_sensor_geometry_summary.csv` | 4 sensörlü geometri sonuçlarının sayısal tablosu. |
| `sensor_count_4_to_10_summary.csv` | Sensör sayısı analizi sayısal tablosu. |

### 11.3. `outputs/03_realistic_los_nlos_noise/`

Bu klasör, `02_baseline_constant_noise` ile aynı çıktı yapısına sahiptir. Farkı, ölçümlerde fabrika engellerine bağlı LOS/NLOS ayrımının kullanılmasıdır.

## 12. Yorumlama

Baseline senaryo, idealize edilmiş düşük gürültülü bir durumdur. Bu sonuçlar sensör geometrisinin temel etkisini anlamak için kullanılır. Realistic LOS/NLOS senaryosu ise duvar ve engellerin ölçüm kalitesini bozduğu daha gerçekçi durumu temsil eder.

Bu nedenle iki klasördeki sonuçlar birlikte yorumlanmalıdır:

| Karşılaştırma | Yorum |
|---|---|
| Baseline iyi, realistic kötü | Sensör geometrisi iyi olsa bile NLOS etkisi sistemi bozuyor olabilir. |
| İki senaryoda da iyi | Sensör geometrisi hem teorik hem gerçekçi koşullarda kararlıdır. |
| Baseline kötü | Sensör geometrisi temel olarak zayıftır. |
| Sensör sayısı artınca hata düşmüyor | Eklenen sensörler geometriyi iyileştirmiyor veya NLOS etkisini artırıyor olabilir. |

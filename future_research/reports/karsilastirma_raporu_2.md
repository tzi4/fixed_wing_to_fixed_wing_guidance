# `goat_gimbal.py` ve `tzi_final.py` Mimari Karşılaştırma Raporu

Bu rapor, `goat_gimbal.py` ve `tzi_final.py` script'leri arasındaki, gerçek hayattaki otonom uçuş performansını, sistem kararlılığını ve gecikme (latency) yönetimini doğrudan etkileyecek **temel mimari ve yapısal farklılıklara** odaklanmaktadır.

---

## 1. Thread (İş Parçacığı) Mimarisi ve Asenkron Çalışma

Gerçek hayatta otonom bir İHA sisteminde en kritik unsurlardan biri, görüntü işleme (vision), otopilot haberleşmesi (MAVLink) ve kontrol algoritmalarının birbirini bloklamadan (non-blocking) asenkron çalışabilmesidir.

### `tzi_final.py` (Gelişmiş Asenkron Mimari)
Bu script, görevleri birbirinden tamamen izole eden çoklu-thread yapısına sahiptir:
*   **`TestCommander` Thread'i:** Sadece otopilota komut göndermekle sorumludur. PID hesaplamalarından bağımsız olarak, sabit bir frekansta (örn. 5 Hz) en son hesaplanan hedefleri (`heading`, `alt`, `throttle`) otopilota iletir.
*   **`_vision_processor` Thread'i:** Sabit 30 Hz frekansta çalışır, en güncel görüntü verisini alır ve PID/PD hesaplamalarını yapar. Sonuçları `TestCommander`'a iletir.
*   **`_mavlink_reader` Thread'i:** Sürekli `recv_match` yaparak pymavlink'in dâhili mesaj önbelleğini (cache) güncel tutar. Diğer thread'ler telemetri verisini okumak istediklerinde beklemeden (non-blocking) bu cache üzerinden (`master.messages.get`) anında okuma yaparlar.
*   **Sonuç:** Kontrol hesaplamaları ve komut gönderimi birbirinden ayrıldığı (decoupled) için, görüntü işlemede yaşanacak anlık bir yavaşlama otopilota komut gitmesini engellemez veya MAVLink hattını gereksiz yere meşgul etmez (network flooding önlenir).

### `goat_gimbal.py` (Kuyruk ve Döngü Tabanlı Mimari)
*   Temel işleyiş tek bir ana döngü (Main Thread) içerisinde gerçekleşir.
*   Görüntü verileri (Redis) ve Telemetri (MAVLink) ayrı thread'lerde okunsa da, PID hesaplamaları ve otopilota komut gönderimi **ana döngüde ardışık (sequential) olarak** yapılır.
*   Komut gönderme frekansı, zaman farkı kontrolüyle (`current_time - self.last_cmd_send_time >= 0.1`) sınırlandırılmıştır.
*   **Sonuç:** Ana döngüdeki herhangi bir gecikme (örneğin log yazma veya veri bekleme), doğrudan komut gönderim frekansını ve kontrol döngüsünü etkileyebilir.

---

## 2. Görüntü (BBox) Verisi İşleme Stratejisi (Latency Yönetimi)

Hedef takibinde gecikme (latency), sistemin kararsızlığa (oscillation) düşmesindeki en büyük nedendir. Hedefin en güncel konumunu işlemek hayati önem taşır.

### `tzi_final.py` (Freshest Frame - En Güncel Kare)
*   Redis'ten gelen mesajlar anında parse edilip thread-safe bir değişken olan `self.latest_bbox`'ın üzerine yazılır (overwrite).
*   Görüntü işleme thread'i her çalıştığında sadece o anki **en güncel** bbox'ı alır.
*   Eğer kontrol döngüsü görüntü gelme hızından yavaşsa, aradaki kareler (frames) **düşürülür (drop)**. Gerçek hayatta hedefin 0.1 saniye önceki konumu yerine şimdiki konumu önemli olduğundan bu en doğru yaklaşımdır.

### `goat_gimbal.py` (Kuyruk / Queue Yapısı)
*   Redis'ten gelen veriler `queue.Queue()` içerisine eklenir. Ana döngü bu kuyruktan verileri sırayla çeker (`self.data_queue.get()`).
*   Eğer ana döngü, görüntülerin geliş hızından daha yavaş çalışırsa kuyrukta veri birikebilir. Bu durumda uçak, hedefin şu anki konumuna değil, *birkaç milisaniye önceki (kuyrukta beklemiş)* konumuna doğru tepki verir. Bu durum yüksek hızlarda salınımlara yol açar.

---

## 3. Hedef Takip Kontrolcüleri: Heading ve Rate Kontrolü

İki sistemin hedefi takip ederkenki (Heading/Yaw) stratejisi gerçek hayatta çok farklı uçuş karakteristikleri ortaya çıkarır.

### `goat_gimbal.py` (Dinamik Dönüş Hızı - Rate Controller)
*   **Heading Rate PID:** `goat_gimbal.py`'nin en güçlü yanlarından biridir. Sadece hedefin hangi açıda (Heading) olduğunu hesaplamakla kalmaz, o açıya **hangi hızla (Heading Rate)** dönülmesi gerektiğini de ayrı bir PID ile (`Kp_rate`, `Kd_rate`) hesaplar.
*   Hata büyükse uçak hedefe hızlı döner, hedefe yaklaştıkça dönüş hızı sönümlenir (düşer).
*   Gerçek hayatta bu yapı; hareketli hedeflere agresif tepki verirken, hedefe kilitlenildiğinde pürüzsüz ve sarsıntısız bir takip sağlar.

### `tzi_final.py` (Sabit Dönüş Hızı - Fixed Rate)
*   Sadece hedeflenen açıyı (Heading) hesaplar.
*   MAVLink `MAV_CMD_GUIDED_CHANGE_HEADING` komutunu gönderirken **sabit bir heading rate (`40` deg/s)** kullanır.
*   Uçak her zaman aynı çevikliğe sahip olmaya çalışır. Hedefe çok yakınken de, çok uzakken de aynı rate limitini kullanması, hassas takiplerde kamerada titremelere (jitter) neden olabilir.

---

## 4. İleri Yön Hızı (Airspeed / Throttle) Kontrolü

Sabit kanatlı bir İHA'nın hedefi takip ederken hedefe yaklaşma veya hedeften uzaklaşma (mesafe koruma) mantığı farklıdır.

### `tzi_final.py` (Gelişmiş Mesafe Koruma - Throttle PID)
*   Hedefin bounding box alanını (açısal büyüklüğünü) referans alarak hedefe olan mesafeyi tahmin eder.
*   Bu büyüklüğe göre ArduPilot'un `TRIM_THROTTLE` parametresini anlık olarak değiştiren **bağımsız bir PID kontrolcüsüne** sahiptir.
*   Hedef çok küçükse (uzaktaysa) throttle artırılır, hedef çok büyükse (yakındaysa) throttle kısılır. Ayrıca sürekli müdahaleyi engellemek için bir **deadzone (ölü bölge)** tanımlanmıştır.
*   Gerçek hayatta bu, hareketli hedefin peşinden koşmayı ve hedefle aradaki mesafeyi otonom olarak korumayı sağlar.

### `goat_gimbal.py` (Sabit Hız / Doğrudan Komut)
*   Mevcut yapıda hızı dinamik olarak mesafeye göre değiştiren bir PID kontrolcüsü bulunmamaktadır.
*   Sabit bir hedef hız (`self.target_airspeed = 20.0`) belirlenir ve MAVLink `MAV_CMD_DO_CHANGE_SPEED` komutu ile doğrudan uçağa iletilir. (Notunuzda belirttiğiniz sequential tuning çalışmalarıyla buranın geliştirildiğini anlıyorum, ancak kodun temelindeki mimari ayrım budur).

---

## Özet ve Gerçek Hayat Etkileri

1.  **Kararlılık ve Salınım (Oscillation):** `tzi_final.py`'nin "Freshest Frame" (Kuyruksuz) veri okuma yapısı, gecikmeyi minimize ettiği için gerçek hayatta salınımları (oscillation) önlemede çok daha başarılı olacaktır. `goat_gimbal.py`'nin kuyruk yapısı yüksek frekanslı işlemlerde risk taşır.
2.  **Uçuş Pürüzsüzlüğü (Smoothness):** `goat_gimbal.py`'nin "Heading Rate PID" mimarisi, hedefe dönüşlerin yumuşaklığı ve hareketli hedef takibi konusunda `tzi_final.py`'nin sabit rate'ine göre bariz bir aerodinamik üstünlük sağlar. Kameradaki görüntünün daha stabil olmasını sağlar.
3.  **İş Yükü Dağılımı (CPU Load & Network):** `tzi_final.py`, `TestCommander` thread'i sayesinde otopilot ile olan iletişimi sınırlandırır ve ana işlemcinin (companion computer) yükünü daha dengeli dağıtır.
4.  **Entegrasyon Önerisi:** Gerçek hayattaki nihai (production) sistemde, **`tzi_final.py`'nin Thread/Cache ve Freshest Frame mimarisi** üzerine, **`goat_gimbal.py`'nin Heading Rate PID** mantığının entegre edilmesi en mükemmel takip performansını (hem gecikmesiz hem de pürüzsüz) verecektir.

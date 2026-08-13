# Otonom Güdüm Yazılımları Mimari Karşılaştırma Raporu

Bu rapor, otonom hedef takibi için geliştirilmiş üç farklı Python dosyasının (`gimballi_roll_pitch.py`, `tzi_final.py` ve `tzi.py`) iletişim (sender) katmanları hariç tutularak, çekirdek mimari, iş parçacığı (thread) yönetimi, kontrol teorisi ve hata toleransı açısından farklılıklarını incelemektedir.

## 1. İş Parçacığı (Thread) Yönetimi ve Veri İletimi

Sistemlerin asenkron veri (görüntü işleme) okuma ve uçuş kontrol döngülerini nasıl ayırdığı en temel mimari farklılıktır.

### `gimballi_roll_pitch.py` Mimarisi
*   **Tam Nesne Yönelimli Thread Yapısı:** `RedisListener` ve `MavlinkManager` sınıfları doğrudan `threading.Thread` üzerinden miras alınarak tasarlanmıştır.
*   **Kuyruk (Queue) Tabanlı İletişim:** Görüntü işleme (Redis) verisi ile ana kontrol döngüsü arasındaki iletişim thread-safe bir `queue.Queue()` yapısı üzerinden sağlanır. Ana döngü, kuyruktan veriyi `timeout=0.05` engellemesi (blocking) ile okur. Bu yaklaşım, aynı hedefin mükerrer işlenmesini önler ve veri yarışını (race condition) ortadan kaldırır.

### `tzi.py` ve `tzi_final.py` Mimarisi
*   **Paylaşımlı Hafıza (Shared Memory) Yaklaşımı:** Sınıf içinde çalıştırılan fonksiyonlar (`_redis_listener`, `_vision_processor`) thread olarak başlatılmıştır. Veri iletimi kuyruk yerine paylaşımlı bellek (ayrı bir değişken olan `self.latest_bbox`) ve bunu koruyan `threading.Lock()` üzerinden yürütülür.
*   **Zamana Dayalı İşleme:** İşleme döngüsü (`_vision_processor`), verinin gelmesini dinlemek yerine sabit $30 \text{ Hz}$ frekansta (`time.sleep(1.0/30.0)`) uyanır ve kilit mekanizmasını açarak en güncel veriyi okur. Eğer veri önceki veri ile aynı zaman etiketine sahipse işlem atlanır.

---

## 2. Kontrol Teorisi (PID Uygulamaları) ve Sinyal İşleme

Kodlar arasındaki en belirgin uçuş karakteristik farkı, geri besleme algoritmalarının olgunluğudur.

### `gimballi_roll_pitch.py`: İleri Düzey PID

Bu sistem doğrudan uçağın Roll (Yatış) ve Pitch (Yunuslama) açılarını hedefler. Daha agresif ancak sinyal bozulmalarına karşı dirençli bir yapıya sahiptir.

*   **Yumuşak Ölü Bölge (Soft Deadzone):** Hedef kameranın merkezine çok yakın olduğunda sistemin aşırı reaksiyon (osilasyon) vermesini engeller. Doğrudan hatayı sıfırlamak yerine kademeli geçiş yapar:
    $$e_{p} = \begin{cases} 0, & \text{if } |e| < \text{deadzone} \\ e - \text{sgn}(e) \cdot \text{deadzone}, & \text{otherwise} \end{cases}$$
*   **Leaky Integrator & Anti-Windup:** 
    *   Hata, ölü bölgeye girdiğinde biriken integral aniden silinmez, yavaşça tahliye edilir ($I = I \times 0.995$).
    *   Çıktı (Roll/Pitch komutu) maksimum sınırlara (doyum) ulaştığında integral şişmesi (windup) önlenerek birikim azaltılır ($I = I \times 0.9$).
*   **Türevsel Filtreleme (Low-Pass Filter):** Görüntü işlemeden kaynaklı ani piksel zıplamalarının türev (D) bileşenini patlatmasını engellemek için $\alpha = 0.03$ katsayılı bir alçak geçiren filtre uygulanır.
    $$D_{new} = (\alpha \cdot D_{raw}) + ((1 - \alpha) \cdot D_{prev})$$

### `tzi.py` ve `tzi_final.py`: Klasik PD/PID

Bu sistemler düşük seviye yatış yerine Yönelim (Heading), İrtifa ve Gaz (Throttle) kontrolü sağlar.

*   **Sade PD Kontrol:** Heading ve İrtifa için Integral (I) parametresi kullanılmaz (Yalnızca P ve D).
*   **Asimetrik İntegral (Throttle):** Yalnızca hedefin alanını (yaklaşmayı) kontrol eden gaz (throttle) PID'sinde integral vardır. Bu integral, matematiksel sönümleme yerine sabit limitlerle sıkıştırılır (Clamp: Min -10, Max +55). Filtre veya leaky yapısı barındırmaz.

---

## 3. Güvenlik ve Failsafe (Hata Toleransı) Mekanizmaları

Görüntü kesintilerinde (hedef kaybı) algoritmaların verdiği tepkiler tamamen farklıdır.

### `gimballi_roll_pitch.py` (Coasting ve Sıfırlama)
*   **Coasting (Süzülme):** Hedef verisi kesildiğinde uçak aniden durmaz. İlk $5$ saniye boyunca araca en son gönderilmiş olan stabil roll/pitch değerleri gönderilmeye devam eder. Bu sayede hedef ağaç arkasına girse bile dönüş yarıçapı korunarak tahmini hedefe yönelim sürdürülür.
*   **Sıfırlama:** Hedef $5$ saniyeden uzun süre bulunamazsa sistem tehlikeyi önlemek için roll ve pitch değerlerini $0.0$'a zorlar (düz uçuş) ve tehlikeli hareketleri engellemek adına PID içindeki tüm geçmiş hata ve integral birikimlerini sıfırlar. Terminalde sürekli bir `[UYARI]` basar.

### `tzi_final.py` (Bekleme ve Heartbeat)
*   Coasting yerine pasif bekleme kullanır. `GUIDED` komutları uçağa gönderilmeyi bırakır (Otopilotun kendi davranışına teslim edilir).
*   Hedef $5$ saniyeden uzun süre yoksa terminale sistemin kilitlenmediğini ve hedef aradığını bildiren `[HEARTBEAT]` logu basar. (PID değerleri sıfırlanmaz).

---

## 4. `tzi_final.py` ile `tzi.py` Arasındaki Ölümcül Yapısal Fark (Startup Bug)

Bu iki kod birbirine çok benzese de, `tzi_final.py`, önceki sürümdeki mantıksal bir hatayı onarmıştır.

*   **`tzi.py` (Hatalı Başlangıç):** Sınıfın `__init__` başlatıcısında `TestCommander` thread'i başlatılırken şu satır yer alır:
    `self.cmd_thread.update(self.test_heading_deg, self.test_alt_m, self.test_throttle_pct)`
    Bu satır, ekranda *hiçbir hedef yokken* uçağa sistem başlar başlamaz varsayılan $50$ metre irtifasına çıkma ve belirli bir hıza gelme emri verir. Drone kalkış anında tehlikeli ve istenmeyen bir şekilde fırlayabilir.
*   **`tzi_final.py` (Güvenli Başlangıç):** `update()` satırı `__init__` içinden kaldırılmıştır. `TestCommander` thread'i başlatılır ancak pasif (`self.active = False`) durumdadır. Sistem ancak ilk `bbox` verisi Redis'ten gelip hesaplamalar yapıldıktan sonra uçağa komut gönderimine başlar. Bu, çok katmanlı otonom sistemlerde elzem olan bir "*Sadece Hedef Varken Hareket Et*" güvenliğidir.

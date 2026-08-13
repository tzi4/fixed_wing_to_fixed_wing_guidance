# 3.1 Pro - İHA Güdüm Sistemleri Kapsamlı Füzyon ve Analiz Raporu

Bu rapor, tarafınızdan sağlanan 5 farklı otonom takip (guidance) kodunun derinlemesine analizini, her birinin havacılık kontrolleri açısından güçlü/zayıf yönlerini ve bu kodların en iyi kısımlarını tek bir çatı altında toplayacak olan **"3.1 Pro"** stratejisini detaylandırmaktadır. 

Sistemden yeni kod yazması istenmediği için bu doküman tamamen teorik mimari, kontrol teorisi analizi ve uygulanabilir yol haritası üzerine odaklanmıştır.

---

## 1. Kod Tabanlarının Derinlemesine Analizi

Elimizdeki 5 kod, mimari (yazılım mühendisliği) ve kontrol algoritması (uçuş dinamiği) olarak iki ayrı "okul" (yaklaşım) barındırıyor: **TZI Okulu** ve **Emir Okulu**.

### A. TZI Okulu (`tzi.py`, `tzi2.1.py`, `tzi_final.py`)
Bu kodların ortak özelliği, **üstün yazılım mimarisine** sahip olmalarıdır.

*   **Mimari Yapı:** Python `threading` kütüphanesi çok efektif kullanılmıştır. Görüntü işleme (`VisionProcessor`), telemetri okuma (`MAVLinkReader`) ve uçuş komutu gönderme (`TestCommander`) birbirinden tamamen izole edilmiş (non-blocking) thread'ler olarak çalışır. Redis'ten gelen veriler (BBox) Pub/Sub mimarisiyle sıfır gecikme ile alınır.
*   **Açısal (Kamera Bağımsız) Kontrol (`tzi_final.py`):** Hedefin piksel hatasını doğrudan kullanmak yerine `math.atan(pixel_error / fx)` formülü ile radyan/derece bazlı **gerçek dünya açılarına** dönüştürür. Bu, kameranın çözünürlüğü veya FOV değeri değişse bile PID kazançlarının (Gain) bozulmamasını sağlar. Otonom sistemlerde bu "altın standarttır".
*   **Hız Kontrol Evrimi:**
    *   `tzi.py`: `TRIM_THROTTLE` parametresini değiştirerek (ArduPilot'un TECS sistemini kandırarak) hız kontrolü yapar. Simülasyonda kusursuz çalışmasının sebebi simülasyon dinamiklerinin bu parametreye anında yanıt vermesidir.
    *   `tzi2.1.py` & `tzi_final.py`: Doğru bir yaklaşımla MAVLink'in `178 (MAV_CMD_DO_CHANGE_SPEED)` komutuna geçmişlerdir.
*   **Failsafe:** `tzi_final.py` içinde hedef 5 saniye kaybolduğunda süzülme (coasting) ve otopilota devretme gibi kritik güvenlik önlemleri hazırdır.
*   **Dezavantajı:** Uçuş dinamiği olarak komutlar basittir. Yönelim ve irtifa komutları doğrudan hedef açı ve hedef irtifa olarak (`MAV_CMD_GUIDED_CHANGE_HEADING` Rate parametresi kullanılmadan) gönderilmektedir. Bu, gerçek rüzgarlı bir havada agresif tepkilere yol açabilir.

### B. Emir Okulu (`kamp_basi_emir.py`, `goat_gimbal.py`)
Bu kodların ortak özelliği, **gerçek uçuş koşullarında kanıtlanmış (flight-validated) kontrol dinamiklerine** sahip olmalarıdır.

*   **Mimari Yapı:** TZI'nin aksine Thread'ler arası iletişimde değişkenler (shared variables) yerine `queue` (kuyruk) mimarisi kullanır. Bu da güvenlidir ancak TZI'nin kilit (Lock) mekanizmalı pub/sub mimarisi gerçek zamanlı kontrolde bir adım daha öndedir.
*   **Kontrol Dinamiği (Rate Controllers):** Bu kodların havada bu kadar iyi çalışmasının asıl sırrı budur. 
    *   **Heading Rate (Dönüş Hızı):** Araca doğrudan "Şu dereceye dön" demek yerine `MAV_CMD_GUIDED_CHANGE_HEADING` komutunun "Rate" parametresini kullanır (Örn: Saniyede 5 derece dön). Bu uçuşun pürüzsüzlüğünü (smoothness) artırır.
    *   **Altitude Rate (İrtifa Hızı):** Aynı şekilde irtifa için `MAV_CMD_GUIDED_CHANGE_ALTITUDE` komutuyla hedef irtifaya giderken "Tırmanma/Alçalma hızını" sınırlandırır.
*   **Filtreleme & Anti-Windup:** PID'nin Türev (Derivative) elemanına ciddi bir **EMA (Exponential Moving Average)** Low-Pass filtresi uygular. Kameradan gelen gürültülü BBox titreşimleri uçağın kanatçıklarını (aileron) yormaz. Ayrıca uç noktalarda Integral birikimini sıfırlayan (Anti-Windup) agresif mantıklar içerir.
*   **Menzil & Hız PID'si:** Hedefin kamera kare (FOV) içindeki kapladığı yüzdeye (Coverage %) bakarak hedefle aradaki mesafeyi tahmin eder ve yaklaşma hızını (Airspeed) hesaplar. Slew rate limiti ile ani hızlanmaları engeller (TECS çökmesini önler).
*   **Loglama:** `FlightLogger` sınıfı ile saniyede birçok kez uçağın Pitch, Roll, Yaw, Error, P-I-D terimleri CSV formatında kaydedilir. 

---

## 2. 3.1 Pro Füzyon Mimarisi

Elimizdeki bu iki güçlü okulu birleştirerek **3.1 Pro** adını vereceğimiz hibrit mimariyi tasarlamalıyız. 

### 3.1 Pro'nun Temel Özellikleri:
1.  **Omurga (Backbone):** Kesinlikle `tzi_final.py`. Sınıf yapısı, failsafe mekanizmaları ve Redis entegrasyonu kusursuz.
2.  **Gözler (Vision):** `tzi_final.py`'nin kamera odak uzaklığına (`fx`, `fy`) dayalı **açısal hata (angular error)** hesaplamaları kullanılmalı. Emir'in piksel tabanlı yaklaşımlarından daha bilimseldir.
3.  **Beyin (Controller):** Emir'in kodundan alınacak **"Rate-Based PID"** (Dönüş ve Tırmanma Hızı PID'si) yaklaşımı, TZI'nin açısal hata hesaplamalarıyla beslenmelidir.
4.  **Kaslar (Actuators):** Komut gönderici `TestCommander`, Emir'in kodundaki gibi Rate (Hız) parametrelerini ArduPilot'a iletecek şekilde güncellenmelidir.
5.  **Hafıza (Logging):** Emir'in CSV `FlightLogger` sınıfı sisteme entegre edilmelidir.

---

## 3. Çevrimdışı (Offline) PID Tuning Planı

Bahsettiğiniz "log toplayıp çevrimdışı analiz yapmak" fikri, uçuş kontrol mühendisliğinde (Flight Control Engineering) **Sistem Tanımlama (System Identification)** olarak geçer. En güvenli ve en profesyonel yöntemdir.

### Adım Adım Offline Tuning Stratejisi:
1.  **Ham Veri Toplama Uçuşu:** 
    *   3.1 Pro füzyon kodu uçağa yüklenir ancak PID kazançları çok "gevşek" (düşük P, sıfır I ve D) bırakılır. Amaç uçağın stabil uçması değil, uçağın "verilen küçük komutlara ne kadar gecikmeyle (delay) ve nasıl tepki verdiğini" (Rise Time, Overshoot) CSV dosyasına (FlightLogger) kaydetmektir.
    *   Havada güvenli irtifadayken hedefe kilitlenilir.
2.  **Sistem Transfer Fonksiyonunun Çıkarılması:**
    *   Uçuştan sonra alınan CSV dosyası MATLAB, Simulink veya Python (SciPy) ortamına aktarılır.
    *   Girdi: `cmd_heading_rate`, Çıktı: `yaw_deg` (Gerçek gerçekleşen dönüş).
    *   Bu iki veri grafiğe dökülerek uçağın matematiksel modeli (Transfer Fonksiyonu) çıkartılır.
3.  **Simüle Edilmiş Tuning (Ziegler-Nichols vb.):**
    *   Çıkartılan bu matematiksel model üzerinde, uçak hiç havalandırılmadan yüzlerce kez PID katsayıları denenir.
    *   En mükemmel P, I ve D değerleri bulunduğunda bu katsayılar `3.1 Pro` koduna gömülür ve uçak 2. uçuşta kusursuz kilitlenmeyi sağlar.

---

## 4. Sonuç ve Öneriler
*   **Mevcut Durum:** `kamp_basi_emir.py`'nin havada çalışması, ArduPilot'un GUIDED modundaki Rate kontrollerini çok iyi manipüle etmesinden kaynaklanmaktadır. TZI ise yazılım mühendisliği açısından çok daha temiz ve hata toleranslıdır (Failsafe).
*   **Tavsiye:** Yazılım ekibinizle bir araya geldiğinizde, `tzi_final.py` iskeletini koruyarak sadece PID fonksiyonu (`guide_aircraft` fonksiyonu) içerisini Emir'in Rate-PID ve EMA filtreleme matematiği ile değiştirmenizi tavsiye ederim. Komut gönderen sınıfa (`TestCommander`) Rate parametrelerini eklemeyi unutmamalısınız.

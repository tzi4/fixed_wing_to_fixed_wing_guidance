# goat_gimbal ham sürümleri

Bu klasör, takımın görüntülü sabit kanat güdüm hattının ilk çalışan ve tam
teşekküllü kaynaklarını tarihsel olarak korur. Kodlar **Tarık Z. İnci**
tarafından geliştirilmiştir.

`tzi.py`, bir sabit kanatlı İHA'nın yalnız kamera hedef kutusunu ve kendi uçuş
telemetrisini kullanarak başka bir sabit kanatlı İHA'ya optik kilit kurması;
hedefi yatay/dikey eksende takip etmesi ve görüntüdeki hedef büyüklüğüne göre
yaklaşması için geliştirilen ilk çalışan bütünleşik koddur. Takımda daha sonra
üretilen görüntülü güdüm kodlarının ana teknik atasıdır.

## Sürümler

| Dosya | Tarihsel rol | Hız kumandası |
| --- | --- | --- |
| `tzi.py` | Ana kaynak; sanal gimbal, heading/irtifa güdümü ve görüntü büyüklüğüne bağlı yaklaşma | `TRIM_THROTTLE` parametresi |
| `tzi2.py` | Aynı yaklaşımın doğrudan hava hızı hedefi kullanan varyantı | `MAV_CMD_GUIDED_CHANGE_SPEED` (43000) |
| `tzi2.1.py` | Standart MAVLink hız komutuna geçirilmiş son ham varyant | `MAV_CMD_DO_CHANGE_SPEED` (178) |
| `goat_gimbal_reference.py` | Daha sonraki `goat_gimbal` hattıyla karşılaştırma için referans | Sabit hava hızı hedefi |
| `trim_throttle_sender.py` | `TRIM_THROTTLE` davranışını tek başına sınamak için yardımcı araç | Parametre yazımı |

## tzi.py ve goat_gimbal ilişkisi

İki kod aynı optik güdüm fikrinin devamıdır: bounding box hatası sanal gimbal
ile dengelenir ve uçağa yön/irtifa hedefleri gönderilir. Operasyonel açıdan en
belirgin ayrım hız kanalıdır. `goat_gimbal`, ayarlanmış bir hava hızı hedefi
gönderirken `tzi.py` hız etkisini `TRIM_THROTTLE` üzerinden üretir.

`tzi.py` ayrıca hedefin görüntüdeki büyüklüğünü kontrol döngüsüne alarak
yaklaşma/mesafe düzenleme davranışını içerir. Arşivlenen ham `goat_gimbal`
sürümünde bu kapalı çevrim yaklaşma katmanı bulunmaz; hava hızı sabit hedef
olarak gönderilir. Bu nedenle `tzi.py`, tarihsel olarak yalnız merkezleme yapan
bir denemeden çok, takip ve yaklaşmayı birlikte çözen daha bütünlüklü ilk takım
uygulamasıdır.

## Arşiv notu

Dosyalar güncel çalışma kopyalarının 13 Ağustos 2026 tarihli anlık görüntüsüdür.
Tarihsel izlenebilirliği korumak için isimleri ve kontrol mantıkları
değiştirilmemiştir. Uçuş öncesinde port, kamera kalibrasyonu, hız/irtifa
sınırları ve ArduPlane komut desteği kullanılan platformda ayrıca
doğrulanmalıdır.

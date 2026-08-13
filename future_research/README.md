# Future Research — görüntülü güdüm araştırma arşivi

Bu klasör, `tzi.py` ile kurulan optik takip temelinden sonra geliştirilen fakat
tamamı yarışma/gerçek uçuş için son ürün hâline getirilmemiş araştırma
kollarının 13 Ağustos 2026 tarihli son anlık görüntüsüdür.

Kodların önemli bölümü çalışır deney düzenekleri içerir; buna rağmen klasörün
tamamı **geliştirme aşamasında** kabul edilmelidir. Parametreler belirli kamera,
uçak ve SITL koşullarına bağlıdır. Doğrudan gerçek uçuşta kullanılmamalıdır.

## Ana araştırma hattı: hedef telemetrisiyle menzil

`guidance/teva.py`, yarışma sunucusundan yaklaşık 1 Hz gelen hedef konumunu ve
avcının kendi `GLOBAL_POSITION_INT` verisini kullanarak iki uçak arasındaki 3B
menzili hesaplar. Hedef örnekleri arasında dead reckoning uygulanır ve bayat
telemetride güvenli biçimde yalnız görüntü tabanlı davranışa dönülür.

Bu araştırmanın bilinçli sınırı önemlidir:

- Hedef telemetrisi yalnızca **menzil bilgisi** olarak, tamamlanan **irtifa
  eksenindeki mesafeye bağlı kontrol** için kullanılır.
- Heading ve hedefin kadrajdaki konumu yalnız kameradan gelir.
- Hedef görünmeden telemetriyle takip yapılmaz.
- Yatay eksen, hız ekseni ve bütünleşik gerçek-uçuş doğrulaması hâlâ geliştirme
  konusudur.

Dolayısıyla burada gelecek vardır: hedef telemetrisinin görüntüyle güvenli
füzyonu, menzile göre kazanç planlama, kapanma hızı yönetimi, yatay eksene
genişleme ve kayıp hedefte güvenli yeniden edinim sonraki çalışmaların doğal
devamıdır.

## Klasörler

| Klasör | İçerik |
| --- | --- |
| `guidance/` | TEVA, fused sürümler, `tzi_final`, `tzi_emir` ve kamp denemeleri |
| `detection/` | Redis `tracker_bbox` üreten renk/SiamRPN ve eski algılama zincirleri |
| `simulation/` | Gazebo/SITL başlatma, formasyon, görev yükleme ve hız yardımcıları |
| `telemetry_tools/` | Sunucu simülatörü, gerçek konum kaydı, menzil kalibrasyonu ve analiz |
| `tests/` | Hedef kilidi, bbox seçimi ve TEVA mantığı birim testleri |
| `reports/` | Füzyon ve kontrol varyantlarının teknik karşılaştırmaları |
| `historical_experiments/` | Sanal gimbal ve önceki kontrol denemeleri |

Aktif ve yeniden üretilebilir Erenimbus SITL ortamı `../bumblebee/` altında
tutulur. Oradaki `teva.py`, araçlar, görev dosyaları ve doğrulama akışı bu
araştırmanın çalışan entegrasyon tarafıdır.

Testleri depo kökünden çalıştırmak için:

```bash
python3 -m unittest \
  bumblebee.test_teva_logic \
  future_research.tests.test_teva_logic \
  future_research.tests.test_guidance_logic
```

## Devir notu

Bu arşiv, Tarık Z. İnci'nin bu araştırma hattındaki son geliştirme teslimidir.
Ham kaynaklar, deney raporları ve destek araçları takımın ileride çalışmayı
sürdürebilmesi için birlikte bırakılmıştır. Gelecekte yapılacak değişikliklerde
deneysel sonuçlarla uçuşta doğrulanmış davranışların ayrı tutulması ve kaynak
atıflarının korunması beklenir.

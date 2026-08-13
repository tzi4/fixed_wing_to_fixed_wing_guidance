# Üçüncü taraf bildirimleri

Bu depo, aşağıda belirtilen üçüncü taraf kaynaklarını içerir veya bunlardan
türetilmiş dosyalar barındırır. Depo geneli lisans GPL-3.0-or-later'dır;
aşağıdaki bileşenlere ait telif ve lisans bildirimleri ayrıca korunur.

## ArduPilot `plane_follow.lua`

`bumblebee/plane_follow.lua`, ArduPilot projesindeki
`libraries/AP_Scripting/applets/plane_follow.lua` dosyasından türetilmiş ve
yerel ihtiyaçlar için değiştirilmiştir.

- Kaynak: <https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Scripting/applets/plane_follow.lua>
- Proje: <https://github.com/ArduPilot/ardupilot>
- Lisans: GNU General Public License v3.0 veya sonrası
- Telif sahipleri: ArduPilot katkıcıları; ayrıntılar upstream geçmişindedir.

Lisans metni `LICENSES/ARDUPILOT-GPL-3.0.txt` dosyasındadır. Bu dosyadaki yerel
değişikliklerin geçmişi bu deponun Git geçmişinde tutulur.

## Intelligent Quads `iq_sim`

Aşağıdaki Gazebo model dosyaları, `Intelligent-Quads/iq_sim` içindeki sabit
kanat model dosyalarının kopyalarını veya değiştirilmiş sürümlerini içerir:

- `bumblebee/models/emir_ucak_temp/`
- `bumblebee/models/hedef_mor/`
- `detectCV_env/gazebo-plane2_model/`

Erenimbus kopyasında kamera ayarları; hedef kopyasında görsel malzemeler ve
model URI'leri değiştirilmiştir. Mesh dosyalarının önemli bir bölümü upstream
ile aynıdır.

- Kaynak: <https://github.com/Intelligent-Quads/iq_sim>
- İncelenen yerel kaynak commit'i: `13e72512cd9748b12a4aa1cac15a2e34b1cdcfd6`
- Lisans: MIT
- Telif: Copyright (c) 2020 Intelligent-Quads

MIT lisans metni `LICENSES/IQ_SIM-MIT.txt` dosyasındadır.

## Haricî bağımlılıklar

ArduPilot, Gazebo Classic eklentisi, ROS, Redis, MAVProxy, QGroundControl ve
Python paketleri bu depoda vendored değildir; kurulum sırasında kendi resmî
kaynaklarından edinilir. Her bağımlılık kendi lisansına tabidir. Sabitlenmiş
kaynak sürümleri için `bumblebee/INSTALL.md` ve `requirements.txt` dosyalarına
bakın.

#!/usr/bin/env python3
"""GUIDED modda heading + irtifa + hiz komutu gonderen test araci.

Eski kodlarin (tzi2.py / final_savasan/speeds.py) gonderme desenini birebir
kullanir:

  * heading  -> MAV_CMD_GUIDED_CHANGE_HEADING  (43002) param1=1 (manyetik burun)
  * irtifa   -> MAV_CMD_GUIDED_CHANGE_ALTITUDE (43001) param7=irtifa
  * hiz      -> MAV_CMD_GUIDED_CHANGE_SPEED    (43000) param1=0 (airspeed)
                basarisiz olursa MAV_CMD_DO_CHANGE_SPEED (178) fallback

IKI CALISMA SEKLI VAR:

1) ARGUMANLI (tek atis, eski davranis, degismedi)
   Komutlar 5 Hz ile tekrar tekrar gonderilir (tzi2.py TestCommander deseni),
   COMMAND_ACK ve STATUSTEXT mesajlari ekrana basilir, telemetri istege bagli
   CSV'ye yazilir.

     ./command_sender.py --connect udpin:127.0.0.1:14553 --sysid 1 \
         --heading 0 --alt 100 --speed 20 --hold 30 --csv reports/x.csv

2) ETKILESIMLI (argumansiz calistirilirsa, ya da --interactive ile)
   Hedefi kameranin onune elle konumlamak icin: bir araca baglanip
   ard arda heading / irtifa / yer hizi sorulur.

     ./command_sender.py

   Akis (SIRALAMA ONEMLI -- once girdi, sonra mod, hemen ardindan komut):
     * "Hangi arac?"  [1] bumblebee (14553)  [2] target (14561), Enter=2
     * mod GUIDED degilse EN BASTA BIR KEZ izin sorulur:
         "Girdiler alininca GUIDED'a alayim mi? [E/h, Enter=E]"
       Cevap SAKLANIR, mod bu noktada DEGISTIRILMEZ.
     * her turda once o anki durum tek satirda basilir, sonra uc soru:
         heading [derece, Enter=su anki]
         irtifa  [m goreli, Enter=su anki]
         yer hizi [m/s, Enter=su anki]
       BOS ENTER = O ANKI DEGER, yani mevcut durumu tutan komut gonderilir.
       Gecersiz/zarf disi girdide soru tekrarlanir.
     * sorulardan sonra (IAS - GPS) ofsetini olcmek icin kisa bir pencere
       ornek toplanir -- ilk turda bu da HENUZ ESKI MODDA, kararli ucusta
       olculur (daha temiz ofset).
     * ANCAK BUNDAN SONRA mod GUIDED'a alinir ve mod dogrulanir
       dogrulanmaz uc komut ART ARDA gonderilip ACK'leri basilir.
     * CIKIS: uc sorudan herhangi birine "q" (veya Ctrl-D) yazmak programdan
       cikarir; ilk turda henuz GUIDED'a gecilmedigi icin ucak ESKI MODUNDA
       kalir. Baska turlu dongu sonsuza kadar tekrarlar (sonraki turlarda
       arac zaten GUIDED'dadir, mod dokunulmaz, istenen rotada ucar).

   NEDEN BU SIRA: ArduPlane hedefsiz GUIDED'a girince mevcut konumun
   etrafinda LOITER'a baslar. Mod once degistirilseydi kullanici sorulara
   cevap yazarken ucak daire cizer, "su anki" varsayilanlar da savrulurdu.
   Girdi once alininca (a) varsayilanlar kararli ucustan okunur (bayat
   heading sorunu hafifler), (b) GUIDED'a gecis ile komut gonderimi
   arasinda insan gecikmesi kalmaz -- ucak dogruca istenen yone doner.

   HIZ = YER HIZI. Sensor yok (ARSPD_TYPE 0) oldugundan hava hizi sentetiktir
   (IAS = |V_gps - W_EKF3|) ve ucak basina kayar; formation.py'deki
   equalise_speed deseni burada da uygulanir: gonderim oncesi kisa bir
   pencerede canli (IAS - GPS) ofseti olculur, komut = istenen yer hizi +
   ofset olarak hesaplanir ve canli AIRSPEED_MIN/MAX zarfina kirpilir.

   NOT: 43000/43001/43002 YALNIZ GUIDED'da kabul edilir (baska modda FAILED),
   43001 irtifasi HOME'a GORELIDIR.
"""

import argparse
import csv
import math
import sys
import time

from pymavlink import mavutil

MAV_CMD_GUIDED_CHANGE_SPEED = 43000
MAV_CMD_GUIDED_CHANGE_ALTITUDE = 43001
MAV_CMD_GUIDED_CHANGE_HEADING = 43002
MAV_CMD_DO_CHANGE_SPEED = 178

SPEED_TYPE = {'airspeed': 0, 'groundspeed': 1}

ACK_NAME = {
    0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
    4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}

# --- etkilesimli mod sabitleri -------------------------------------------
# (isim, baglanti, beklenen SysID, kabul edilen YER hizi araligi)
# Yer hizi araliklari formation.py SPEED_ENVELOPE ile ayni; canli
# AIRSPEED_MIN/MAX okunabiliyorsa gonderim oncesi ayrica onunla kirpilir.
VEHICLES = {
    '1': ('bumblebee', 'udpin:127.0.0.1:14553', 1, (15.0, 24.0)),
    '2': ('target', 'udpin:127.0.0.1:14561', 2, (9.0, 22.0)),
}
DEFAULT_VEHICLE = '2'          # kullanim senaryosu: hedefi elle konumlamak
ALT_RANGE = (0.0, 1000.0)      # m, HOME'a goreli
HEADING_RANGE = (0.0, 360.0)   # derece
QUIT_WORDS = ('q', 'Q', 'cik', 'çık', 'exit', 'quit')

QUIT = object()                # prompt_value(): kullanici cikmak istedi


def ack_text(result):
    return ACK_NAME.get(result, f'RESULT_{result}')


class CommandSender:
    """Tek araca heading/irtifa/hiz komutu gonderen ince sarmalayici."""

    def __init__(self, connect, sysid, timeout=30.0):
        self.master = mavutil.mavlink_connection(
            connect, source_system=254, source_component=190)
        deadline = time.monotonic() + timeout
        heartbeat = None
        while time.monotonic() < deadline:
            candidate = self.master.recv_match(
                type='HEARTBEAT', blocking=True, timeout=1)
            if candidate is not None and (sysid is None
                                          or candidate.get_srcSystem() == sysid):
                heartbeat = candidate
                break
        if heartbeat is None:
            raise RuntimeError(f'{connect}: SysID {sysid} heartbeat alinamadi')
        self.master.target_system = heartbeat.get_srcSystem()
        self.master.target_component = heartbeat.get_srcComponent()
        self.acks = []          # (t, command, result)
        self.statustexts = []   # (t, text)
        self.t0 = time.monotonic()

    # ---------------- gonderme ----------------
    def _long(self, command, *params):
        padded = list(params) + [0.0] * (7 - len(params))
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            command, 0, *padded)

    def send_heading(self, heading_deg, heading_rate=40.0):
        """param1=1 raw manyetik heading, param2=derece, param3=deg/s (0 olamaz)."""
        self._long(MAV_CMD_GUIDED_CHANGE_HEADING,
                   1, float(heading_deg % 360.0), float(heading_rate))

    def send_altitude(self, alt_m, climb_rate=0.0):
        """param3=tirmanis hizi (0=maksimum), param7=hedef irtifa.

        COMMAND_LONG->COMMAND_INT donusumu MAV_FRAME_GLOBAL_RELATIVE_ALT
        varsaydigi icin irtifa HOME'a GORE metredir (AMSL degil).
        """
        self._long(MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                   0, 0, float(climb_rate), 0, 0, 0, float(alt_m))

    def send_speed_43000(self, speed_ms, speed_type=0, accel=1.0):
        """Sam Hyams yontemi: sadece GUIDED'da gecerli, zarf disi -> FAILED."""
        self._long(MAV_CMD_GUIDED_CHANGE_SPEED,
                   float(speed_type), float(speed_ms), float(accel))

    def send_speed_178(self, speed_ms, speed_type=0, throttle=-1.0):
        """final_savasan/speeds.py deseni: DO_CHANGE_SPEED, param3=-1 (gaz degismez)."""
        self._long(MAV_CMD_DO_CHANGE_SPEED,
                   float(speed_type), float(speed_ms), float(throttle))

    # ---------------- mod ----------------
    def mode(self):
        hb = self.master.messages.get('HEARTBEAT')
        return mavutil.mode_string_v10(hb) if hb else None

    def set_mode(self, name, timeout=15.0):
        mapping = self.master.mode_mapping() or {}
        mode_id = mapping.get(name)
        if mode_id is None:
            raise RuntimeError(f'{name} modu bulunamadi')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.master.set_mode(mode_id)
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if msg is not None and getattr(msg, 'custom_mode', None) == mode_id:
                return True
        return False

    # ---------------- okuma ----------------
    def pump(self, verbose=True):
        """Gelen mesajlari bosalt; ACK ve STATUSTEXT'leri kaydet/bas."""
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                return
            kind = msg.get_type()
            if kind == 'COMMAND_ACK':
                row = (time.monotonic() - self.t0, msg.command, msg.result)
                self.acks.append(row)
                if verbose:
                    print(f'  ACK cmd={msg.command} -> {ack_text(msg.result)}')
            elif kind == 'STATUSTEXT':
                text = msg.text.strip()
                self.statustexts.append((time.monotonic() - self.t0, text))
                if verbose:
                    print(f'  TEXT: {text}')

    def telemetry(self):
        """(indicated_airspeed, groundspeed, gps_speed, alt_rel, hdg, throttle)."""
        vfr = self.master.messages.get('VFR_HUD')
        gpi = self.master.messages.get('GLOBAL_POSITION_INT')
        gps_speed = None
        alt_rel = None
        if gpi is not None:
            gps_speed = math.hypot(gpi.vx, gpi.vy) / 100.0
            alt_rel = gpi.relative_alt / 1000.0
        return (
            getattr(vfr, 'airspeed', None),
            getattr(vfr, 'groundspeed', None),
            gps_speed,
            alt_rel,
            getattr(vfr, 'heading', None),
            getattr(vfr, 'throttle', None),
        )

    def request_streams(self, rate_hz=10):
        for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD,
                       mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                       mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE):
            self._long(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                       msg_id, int(1e6 / rate_hz))

    # ---------------- etkilesimli mod yardimcilari ----------------
    def drain(self, verbose=False):
        """Soru sorulurken biriken UDP yigilmasini bosalt (bayat telemetri)."""
        self.pump(verbose=verbose)

    def sample(self, seconds, verbose=False):
        """Kisa pencerede ortalama telemetri olc (formation.sample_speeds deseni).

        Once birikmis mesajlar atilir, sonra `seconds` boyunca TAZE VFR_HUD /
        GLOBAL_POSITION_INT ornekleri toplanir.

        Doner: {'ias', 'gps', 'alt', 'hdg'} (olculemeyen alan None).
        """
        self.drain(verbose=verbose)
        ias, gps, alt, hdg = [], [], [], []
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type=['VFR_HUD', 'GLOBAL_POSITION_INT', 'COMMAND_ACK',
                      'STATUSTEXT'],
                blocking=True, timeout=1)
            if msg is None:
                continue
            kind = msg.get_type()
            if kind == 'VFR_HUD':
                ias.append(msg.airspeed)
                hdg.append(msg.heading)
            elif kind == 'GLOBAL_POSITION_INT':
                gps.append(math.hypot(msg.vx, msg.vy) / 100.0)
                alt.append(msg.relative_alt / 1000.0)
            elif kind == 'COMMAND_ACK':
                self.acks.append(
                    (time.monotonic() - self.t0, msg.command, msg.result))
            elif kind == 'STATUSTEXT':
                text = msg.text.strip()
                self.statustexts.append((time.monotonic() - self.t0, text))
                if verbose:
                    print(f'  TEXT: {text}')

        def mean(values):
            return sum(values) / len(values) if values else None

        return {'ias': mean(ias), 'gps': mean(gps), 'alt': mean(alt),
                'hdg': hdg[-1] if hdg else None}

    def read_param(self, name, timeout=6.0):
        """Tek parametreyi oku (formation.read_param); alinamazsa None."""
        self.master.mav.param_request_read_send(
            self.master.target_system, self.master.target_component,
            name.encode(), -1)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True,
                                         timeout=1)
            if msg is not None and msg.param_id == name:
                return float(msg.param_value)
        return None

    def wait_ack(self, command, timeout=3.0):
        """Belirli komutun ACK'ini bekle; gelmezse None."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self.master.recv_match(
                type=['COMMAND_ACK', 'STATUSTEXT'], blocking=True, timeout=1)
            if msg is None:
                continue
            if msg.get_type() == 'STATUSTEXT':
                self.statustexts.append(
                    (time.monotonic() - self.t0, msg.text.strip()))
                continue
            self.acks.append((time.monotonic() - self.t0, msg.command,
                              msg.result))
            if msg.command == command:
                return msg.result
        return None

    def close(self):
        self.master.close()


# ============================ ETKILESIMLI MOD ============================
# Soru katmani saf fonksiyon: reader/out enjekte edilebilir (birim testi).

def prompt_value(label, unit, default, low, high, reader=input, out=sys.stderr):
    """Sayi sor. Bos Enter -> default (= su anki deger), 'q' -> QUIT.

    Sayi olmayan girdi ya da [low, high] disi deger uyari basip soruyu
    TEKRARLATIR. default None ise (telemetri okunamadi) bos Enter kabul
    edilmez. EOF (Ctrl-D) -> QUIT.
    """
    if default is None:
        question = f'{label} [{unit}, su anki okunamadi, deger girin]: '
    else:
        question = f'{label} [{unit}, Enter={default:g} (su anki)]: '
    while True:
        try:
            raw = reader(question)
        except EOFError:
            print('  girdi bitti -> cikiliyor', file=out)
            return QUIT
        raw = raw.strip()
        if raw in QUIT_WORDS:
            return QUIT
        raw = raw.replace(',', '.')
        if not raw:
            if default is None:
                print('  su anki deger okunamadi, bir sayi girin', file=out)
                continue
            return default
        try:
            value = float(raw)
        except ValueError:
            print('  gecersiz girdi (sayi bekleniyordu), tekrar deneyin',
                  file=out)
            continue
        if not low <= value <= high:
            print(f'  {value:g} kabul araligi disinda [{low:g}, {high:g}], '
                  f'tekrar deneyin', file=out)
            continue
        return value


def prompt_vehicle(reader=input, out=sys.stderr, default=DEFAULT_VEHICLE):
    """Arac sec: '1' bumblebee, '2' target, bos Enter -> default. 'q' -> QUIT."""
    options = '  '.join(
        f'[{key}] {VEHICLES[key][0]} ({VEHICLES[key][1].rsplit(":", 1)[1]})'
        for key in sorted(VEHICLES))
    question = f'Hangi arac? {options}, Enter={default}: '
    while True:
        try:
            raw = reader(question)
        except EOFError:
            print('  girdi bitti -> cikiliyor', file=out)
            return QUIT
        raw = raw.strip()
        if raw in QUIT_WORDS:
            return QUIT
        if not raw:
            return default
        if raw in VEHICLES:
            return raw
        by_name = [key for key, spec in VEHICLES.items()
                   if spec[0].lower() == raw.lower()]
        if by_name:
            return by_name[0]
        print(f'  gecersiz secim "{raw}", {sorted(VEHICLES)} bekleniyor',
              file=out)


def prompt_yes_no(question, default=True, reader=input, out=sys.stderr):
    """E/h sorusu. Bos Enter -> default. EOF -> default."""
    suffix = '[E/h, Enter=E]' if default else '[e/H, Enter=H]'
    while True:
        try:
            raw = reader(f'{question} {suffix}: ')
        except EOFError:
            print(f'  girdi bitti, varsayilan {"E" if default else "H"} '
                  f'kullanildi', file=out)
            return default
        raw = raw.strip().lower()
        if not raw:
            return default
        if raw in ('e', 'evet', 'y', 'yes'):
            return True
        if raw in ('h', 'hayir', 'hayır', 'n', 'no'):
            return False
        print('  gecersiz girdi (e/h bekleniyordu), tekrar deneyin', file=out)


def status_line(snap, mode):
    """Tek satirlik durum ozeti."""
    def fmt(value, digits=1, suffix=''):
        return '?' if value is None else f'{value:.{digits}f}{suffix}'
    return (f'su an: mod={mode} hdg={"?" if snap["hdg"] is None else snap["hdg"]}'
            f' alt={fmt(snap["alt"])}m (goreli)'
            f' IAS={fmt(snap["ias"])} GPS={fmt(snap["gps"])}')


def interactive(args):
    """Argumansiz (veya --interactive) calisma: soru-cevap komut dongusu."""
    choice = prompt_vehicle()
    if choice is QUIT:
        print('cikildi')
        return 0
    name, connect, sysid, envelope = VEHICLES[choice]

    print(f'[baglaniyor] {name} {connect} (SysID {sysid}) ...')
    try:
        sender = CommandSender(connect, sysid, timeout=args.connect_timeout)
    except OSError as exc:
        print(f'HATA: {connect} baglanti kurulamadi ({exc}). Port baska bir '
              f'surec tarafindan bind edilmis olabilir '
              f'(goat_gimbal_ucak.py 14553 kullanir).', file=sys.stderr)
        return 1
    except RuntimeError as exc:
        print(f'HATA: {exc}', file=sys.stderr)
        return 1

    try:
        sender.request_streams()
        mode = sender.mode()
        print(f'[bagli] {name} sys={sender.master.target_system} '
              f'comp={sender.master.target_component} mod={mode}')

        # --- GUIDED izni: SORULUR ama mod SIMDI DEGISTIRILMEZ ---
        # Hedefsiz GUIDED = mevcut konum etrafinda LOITER; sorulari
        # cevaplarken ucak daire cizmesin diye mod degisimi, girdiler
        # alindiktan sonraki gonderim adimina ertelenir.
        allow_guided = True
        if mode != 'GUIDED':
            print(f'UYARI: mod {mode}; 43000/43001/43002 yalniz GUIDED\'da '
                  f'kabul edilir.', file=sys.stderr)
            print('Mod HEMEN degistirilmeyecek: once sorular, sonra GUIDED, '
                  'hemen ardindan komutlar (hedefsiz GUIDED = LOITER).')
            allow_guided = prompt_yes_no(
                "Girdiler alininca GUIDED'a alayim mi?", default=True)
            if not allow_guided:
                print('[mod] degistirilmeyecek; komutlar FAILED donebilir',
                      file=sys.stderr)

        # --- canli zarf (bir kez okunur) ---
        low = sender.read_param('AIRSPEED_MIN')
        high = sender.read_param('AIRSPEED_MAX')
        if low is None or high is None:
            low, high = envelope
            print(f'[zarf] AIRSPEED_MIN/MAX okunamadi, tablo degeri '
                  f'[{low:g}, {high:g}] kullanilacak', file=sys.stderr)
        else:
            print(f'[zarf] canli AIRSPEED_MIN={low:g} AIRSPEED_MAX={high:g} '
                  f'(IAS komutu buna kirpilir)')
        gs_low, gs_high = envelope

        print("\nBos Enter = O ANKI deger. Cikis icin herhangi bir soruya 'q'.")
        print('Sira: sorular -> (IAS-GPS) ofsetu -> GUIDED -> komutlar '
              '(mod, girdiler alinmadan degistirilmez).\n')

        # --- komut dongusu ---
        while True:
            snap = sender.sample(args.status_sample)
            mode = sender.mode()
            print(status_line(snap, mode))

            hdg_now = None if snap['hdg'] is None else float(snap['hdg'])
            heading = prompt_value('heading', 'derece', hdg_now,
                                   HEADING_RANGE[0], HEADING_RANGE[1])
            if heading is QUIT:
                break
            altitude = prompt_value('irtifa', 'm goreli', snap['alt'],
                                    ALT_RANGE[0], ALT_RANGE[1])
            if altitude is QUIT:
                break
            wish_gps = prompt_value('yer hizi', 'm/s', snap['gps'],
                                    gs_low, gs_high)
            if wish_gps is QUIT:
                break

            # Hiz: formation.equalise_speed deseni -> canli (IAS-GPS) ofseti
            print(f'  ({args.offset_sample:g} s ofset olcumu ...)')
            measured = sender.sample(args.offset_sample)
            if measured['ias'] is None or measured['gps'] is None:
                offset = 0.0
                print('  UYARI: IAS/GPS ornegi alinamadi, ofset 0 varsayildi',
                      file=sys.stderr)
            else:
                offset = measured['ias'] - measured['gps']
            wanted = wish_gps + offset
            clamped = min(max(wanted, low), high)
            if abs(clamped - wanted) > 1e-6:
                # Zarf disi komut KIRPILMAZ, REDDEDILIR (Plane::do_change_speed)
                print(f'  UYARI: {wanted:.2f} zarf disi [{low:g}, {high:g}]; '
                      f'{clamped:.2f} gonderiliyor', file=sys.stderr)
            wanted = clamped

            # --- mod: girdiler alindi, GUIDED'a ANCAK SIMDI geciliyor ---
            # Boylece GUIDED'a girisle komut gonderimi arasinda insan
            # gecikmesi yok; ucak hedefsiz LOITER'a oturmadan yon aliyor.
            mode = sender.mode()
            if mode != 'GUIDED':
                if allow_guided:
                    print("  (GUIDED'a aliniyor ...)")
                    if sender.set_mode('GUIDED'):
                        print('[mod] GUIDED onaylandi')
                    else:
                        print('[mod] GUIDED DEGISTIRILEMEDI; komutlar FAILED '
                              'donecek', file=sys.stderr)
                else:
                    print(f'[mod] {mode} korunuyor (izin verilmedi); komutlar '
                          f'FAILED donebilir', file=sys.stderr)

            sender.drain()
            sender.send_heading(heading, args.heading_rate)
            ack_hdg = sender.wait_ack(MAV_CMD_GUIDED_CHANGE_HEADING,
                                      args.ack_timeout)
            sender.send_altitude(altitude, args.climb_rate)
            ack_alt = sender.wait_ack(MAV_CMD_GUIDED_CHANGE_ALTITUDE,
                                      args.ack_timeout)
            sender.send_speed_43000(wanted, SPEED_TYPE['airspeed'], args.accel)
            ack_spd = sender.wait_ack(MAV_CMD_GUIDED_CHANGE_SPEED,
                                      args.ack_timeout)

            def label(result):
                return 'ACK YOK' if result is None else ack_text(result)

            print(f'-> heading {heading:g} deg      : {label(ack_hdg)}')
            print(f'-> irtifa  {altitude:g} m (gor.): {label(ack_alt)}')
            print(f'-> IAS {wanted:.1f} komutlandi (istenen yer hizi '
                  f'{wish_gps:g}, ofset {offset:+.2f}): {label(ack_spd)}')
            print()
    except KeyboardInterrupt:
        print('\n(Ctrl-C) cikiliyor')
    finally:
        sender.close()
    print('cikildi')
    return 0


def run(args):
    sender = CommandSender(args.connect, args.sysid)
    print(f'[bagli] {args.connect} sys={sender.master.target_system} '
          f'comp={sender.master.target_component} mod={sender.mode()}')
    sender.request_streams()

    if args.mode:
        if sender.set_mode(args.mode):
            print(f'[mod] {args.mode} onaylandi')
        else:
            print(f'[mod] {args.mode} DEGISTIRILEMEDI', file=sys.stderr)

    speed_type = SPEED_TYPE[args.speed_type]
    method = args.method
    writer = None
    handle = None
    if args.csv:
        handle = open(args.csv, 'w', newline='')
        writer = csv.writer(handle)
        writer.writerow(['t_s', 'cmd_speed', 'indicated_as', 'groundspeed',
                         'gps_speed', 'alt_rel', 'heading', 'throttle', 'mode'])

    period = 1.0 / args.rate
    end = time.monotonic() + args.hold
    next_print = 0.0
    fallback_done = False
    try:
        while time.monotonic() < end:
            loop_t = time.monotonic() - sender.t0
            if args.heading is not None:
                sender.send_heading(args.heading, args.heading_rate)
            if args.alt is not None:
                sender.send_altitude(args.alt, args.climb_rate)
            if args.speed is not None:
                if method in ('auto', '43000'):
                    sender.send_speed_43000(args.speed, speed_type, args.accel)
                if method == '178':
                    sender.send_speed_178(args.speed, speed_type)
            sender.pump(verbose=not args.quiet)

            # auto: 43000 reddedilirse 178'e dus
            if method == 'auto' and not fallback_done:
                bad = [r for t, c, r in sender.acks
                       if c == MAV_CMD_GUIDED_CHANGE_SPEED and r != 0]
                if bad:
                    print(f'[fallback] 43000 -> {ack_text(bad[-1])}, '
                          f'178 DO_CHANGE_SPEED deneniyor')
                    method = '178'
                    fallback_done = True

            ias, gs, gps, alt, hdg, thr = sender.telemetry()
            if writer:
                writer.writerow([f'{loop_t:.2f}', args.speed, ias, gs, gps,
                                 alt, hdg, thr, sender.mode()])
            if loop_t >= next_print:
                next_print = loop_t + 1.0
                print(f'  t={loop_t:6.1f} mod={sender.mode()} '
                      f'IAS={ias if ias is None else round(ias, 2)} '
                      f'GS={gps if gps is None else round(gps, 2)} '
                      f'alt={alt if alt is None else round(alt, 1)} '
                      f'hdg={hdg} thr={thr}')
            time.sleep(period)
    finally:
        if handle:
            handle.close()
        sender.close()

    print('\n--- ACK ozeti ---')
    seen = {}
    for _, command, result in sender.acks:
        seen.setdefault((command, result), 0)
        seen[(command, result)] += 1
    for (command, result), count in sorted(seen.items()):
        print(f'  cmd {command}: {ack_text(result)} x{count}')
    if not sender.acks:
        print('  (hic ACK gelmedi)')
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--connect', default='udpin:127.0.0.1:14553')
    parser.add_argument('--sysid', type=int, default=None)
    parser.add_argument('--heading', type=float, default=None, help='derece 0-360')
    parser.add_argument('--alt', type=float, default=None, help='metre (home-relative)')
    parser.add_argument('--speed', type=float, default=None, help='m/s')
    parser.add_argument('--speed-type', choices=sorted(SPEED_TYPE), default='airspeed')
    parser.add_argument('--method', choices=('auto', '43000', '178'), default='auto')
    parser.add_argument('--accel', type=float, default=1.0, help='43000 param3 (m/s2)')
    parser.add_argument('--heading-rate', type=float, default=40.0, help='deg/s')
    parser.add_argument('--climb-rate', type=float, default=0.0, help='m/s, 0=maks')
    parser.add_argument('--mode', default='GUIDED', help='"" verilirse mod degistirilmez')
    parser.add_argument('--hold', type=float, default=30.0, help='saniye')
    parser.add_argument('--rate', type=float, default=5.0, help='gonderme Hz')
    parser.add_argument('--csv', default=None)
    parser.add_argument('--quiet', action='store_true')
    # --- etkilesimli mod ---
    parser.add_argument('--interactive', action='store_true',
                        help='soru-cevap modu (argumansiz calistirmakla ayni): '
                             'once sorular, sonra GUIDED, hemen ardindan komut')
    parser.add_argument('--status-sample', type=float, default=1.5,
                        help='etkilesimli: durum satiri icin olcum penceresi (s)')
    parser.add_argument('--offset-sample', type=float, default=4.0,
                        help='etkilesimli: (IAS-GPS) ofset olcum penceresi (s)')
    parser.add_argument('--ack-timeout', type=float, default=3.0,
                        help='etkilesimli: komut basina ACK bekleme suresi (s)')
    parser.add_argument('--connect-timeout', type=float, default=30.0,
                        help='etkilesimli: heartbeat bekleme suresi (s)')
    args = parser.parse_args()
    if args.rate <= 0 or args.hold < 0:
        parser.error('rate > 0 ve hold >= 0 olmali')
    if args.status_sample <= 0 or args.offset_sample <= 0:
        parser.error('--status-sample ve --offset-sample > 0 olmali')
    if args.ack_timeout < 0 or args.connect_timeout <= 0:
        parser.error('--ack-timeout >= 0 ve --connect-timeout > 0 olmali')
    # Argumansiz cagri (ya da acik --interactive) -> soru-cevap modu.
    if args.interactive or len(sys.argv) == 1:
        return interactive(args)
    return run(args)


if __name__ == '__main__':
    raise SystemExit(main())

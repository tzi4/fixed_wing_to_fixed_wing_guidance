"""
Uçak parametrelerini .parm dosyalarından okuyup SITL'e gönderen script.

Mimari:
  plane1.parm  ->  Instance 0 (SYSID 1)  ->  tcp:127.0.0.1:5762
  plane2.parm  ->  Instance 1 (SYSID 2)  ->  tcp:127.0.0.1:5772

Kullanım:
  python3 set_params.py          (interaktif mod - terminal sorar)
  python3 set_params.py 1        (sadece Plane 1)
  python3 set_params.py 2        (sadece Plane 2)
  python3 set_params.py 3        (her iki uçak)
"""
import os
import sys
from pymavlink import mavutil

# --- HER UÇAGIN TANIMI ---
AIRCRAFT = [
    {
        "name": "Plane 1",
        "parm_file": "plane1.parm",
        "port": "tcp:127.0.0.1:5762",   # Instance 0
    },
    {
        "name": "Plane 2",
        "parm_file": "plane2.parm",
        "port": "tcp:127.0.0.1:5772",   # Instance 1
    },
]


def parse_parm_file(filepath, run_mode):
    """
    .parm dosyasını okur, (param_adı, değer) listesi döner.
    # ile başlayan satırlar ve boş satırlar atlanır.
    """
    params = []
    if not os.path.isfile(filepath):
        print(f"  ⚠ Dosya bulunamadı: {filepath}")
        return params

    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 2:
                param_name = parts[0]
                if run_mode == "1" and param_name == "AIRSPEED_CRUISE":
                    continue
                if run_mode == "2" and param_name == "TRIM_THROTTLE":
                    continue
                try:
                    param_value = float(parts[1])
                    params.append((param_name, param_value))
                except ValueError:
                    print(f"  ⚠ Geçersiz değer atlandı: {line}")
    return params


def apply_params(port, params, timeout=15):
    """
    Belirtilen porta bağlanıp parametre listesini gönderir.
    """
    try:
        print(f"  Bağlanılıyor: {port} ...")
        master = mavutil.mavlink_connection(port)
        master.wait_heartbeat(timeout=timeout)
        sysid = master.target_system
        print(f"  ✓ Heartbeat alındı (SYSID={sysid})")

        for param_name, param_value in params:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                param_name.encode('utf-8'),
                float(param_value),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )

            # Onay bekle
            ack = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if ack and ack.param_id.strip('\x00') == param_name:
                print(f"  ✓ {param_name} = {ack.param_value}")
            else:
                print(f"  ⚠ {param_name} = {param_value} (onay alınamadı ama gönderildi)")

        master.close()
        return True
    except Exception as e:
        print(f"  ✗ HATA: {e}")
        return False


def get_run_mode():
    """Terminalden mod seçimini alır."""
    while True:
        print("\nHangi mod:")
        print("  1 - trim_throttle (Şu anki işleyiş, TRIM_THROTTLE kullanılır)")
        print("  2 - airspeed (AIRSPEED_CRUISE parametresi gönderilir)")
        ans = input("\nSeçim (1/2): ").strip()
        if ans in ['1', '2']:
            return ans
        print("⚠ Lütfen 1 veya 2 girin.")


def get_choice():
    """Terminalde veya argümandan seçim al."""
    # Komut satırı argümanı varsa onu kullan
    if len(sys.argv) > 1:
        return sys.argv[1]

    # Yoksa kullanıcıya sor
    print("\nHedef seçin:")
    print("  1 - Sadece Plane 1")
    print("  2 - Sadece Plane 2")
    print("  3 - Her iki uçak")
    return input("\nSeçim (1/2/3): ").strip()


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 50)
    print("  UCAK PARAMETRE AYARLAYICI")
    print("=" * 50)

    run_mode = get_run_mode()
    choice = get_choice()

    if choice == "1":
        targets = [AIRCRAFT[0]]
    elif choice == "2":
        targets = [AIRCRAFT[1]]
    elif choice == "3":
        targets = AIRCRAFT
    else:
        print(f"\n✗ Geçersiz seçim: '{choice}' (1, 2 veya 3 girin)")
        sys.exit(1)

    for ac in targets:
        parm_path = os.path.join(script_dir, ac["parm_file"])
        print(f"\n--- {ac['name']} ({ac['parm_file']}) ---")

        params = parse_parm_file(parm_path, run_mode)
        if not params:
            print("  Parametresiz veya dosya bulunamadı, atlanıyor.")
            continue

        print(f"  {len(params)} parametre okundu:")
        for name, val in params:
            print(f"    {name} = {val}")

        apply_params(ac["port"], params)

    print("\n" + "=" * 50)
    print("  TAMAMLANDI")
    print("=" * 50)

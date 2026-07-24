#!/usr/bin/env python3
"""VNC/X11 uzerinden arayuzsiz, PySerial ile dogrudan Arduino Mega motor kontrolu ve P tuslu Fotoğraf Cekici (ROS-SIZ)."""

import argparse
import os
import select
import shutil
import subprocess
import sys
import time
from datetime import datetime

try:
    import serial
except ImportError:
    print("HATA: sudo apt install python3-serial", file=sys.stderr)
    raise SystemExit(1)

try:
    from Xlib import XK, display
except ImportError:
    print("HATA: sudo apt install python3-xlib", file=sys.stderr)
    raise SystemExit(1)


TUS_ADLARI = {
    "w": "ILERI",
    "s": "GERI",
    "a": "SOLA DON",
    "d": "SAGA DON",
}

KLAVYE_YOKLAMA_S = 0.020


def argumanlari_oku():
    parser = argparse.ArgumentParser(
        description="VNC/X11 uzerinden arayuzsiz WASD rover kontrolu ve Fotoğraf Cekici (ROS-siz)"
    )
    parser.add_argument("--port", default="/dev/ttyACM0", help="Arduino Mega seri portu")
    parser.add_argument("--baud", type=int, default=115200, help="Seri port baud hizi")
    parser.add_argument("--forward-pwm", type=int, default=80, help="Ileri hareket PWM hizi (0-255)")
    parser.add_argument("--back-pwm", type=int, default=80, help="Geri hareket PWM hizi (0-255)")
    parser.add_argument("--turn-pwm", type=int, default=60, help="Donus hareket PWM hizi (0-255)")
    parser.add_argument("--repeat-ms", type=int, default=100, help="Komut tekrar araligi ms")
    parser.add_argument("--photo-dir", default=os.path.expanduser("~/rover_photos"), help="Fotograf kayit dizini")
    parser.add_argument("--photo-width", type=int, default=1920, help="Fotograf genisligi piksel")
    parser.add_argument("--photo-height", type=int, default=1080, help="Fotograf yuksekligi piksel")
    parser.add_argument("--photo-timeout", type=int, default=500, help="Kamera pozlama suresi ms")
    parser.add_argument(
        "--display",
        help="X11 ekrani; VNC terminalinde genellikle otomatik bulunur (ornek :1)",
    )
    parser.add_argument("--verbose", action="store_true", help="Detayli seri port ciktilari")
    args = parser.parse_args()

    for ad in ("forward_pwm", "back_pwm", "turn_pwm"):
        if not 0 <= getattr(args, ad) <= 255:
            parser.error(f"--{ad.replace('_', '-')} 0 ile 255 arasinda olmali")
    if not 20 <= args.repeat_ms <= 500:
        parser.error("--repeat-ms 20 ile 500 arasinda olmali")

    return args


def x11_ekranini_ac(ekran_adi=None):
    try:
        ekran = display.Display(ekran_adi)
    except Exception as hata:
        raise RuntimeError(
            "VNC/X11 masaustune baglanilamadi. Programi VNC icinde actigin "
            "terminalden calistir ve 'echo $DISPLAY' ciktisini kontrol et."
        ) from hata

    tus_sembolleri = {
        "w": "w",
        "s": "s",
        "a": "a",
        "d": "d",
        "p": "p",
        "esc": "Escape",
        "ctrl_sol": "Control_L",
        "ctrl_sag": "Control_R",
        "c": "c",
    }
    tus_kodlari = {
        ad: ekran.keysym_to_keycode(XK.string_to_keysym(sembol))
        for ad, sembol in tus_sembolleri.items()
    }

    eksik = [ad for ad, kod in tus_kodlari.items() if kod == 0]
    if eksik:
        ekran.close()
        raise RuntimeError(
            "X11 klavye eslemesinde tus bulunamadi: " + ", ".join(eksik)
        )

    return ekran, tus_kodlari


def x11_basili_tuslari_oku(ekran, tus_kodlari):
    try:
        durum = ekran.query_keymap()
    except Exception as hata:
        raise RuntimeError("VNC/X11 klavye baglantisi kesildi.") from hata

    def basili(kod):
        return bool(durum[kod >> 3] & (1 << (kod & 7)))

    return {ad for ad, kod in tus_kodlari.items() if basili(kod)}


def x11_odagini_oku(ekran):
    try:
        odak = ekran.get_input_focus().focus
    except Exception as hata:
        raise RuntimeError("VNC/X11 pencere odagi okunamadi.") from hata
    return getattr(odak, "id", odak)


def komut_gonder(ser, komut):
    ser.write((komut + "\n").encode("ascii"))


def seri_cevaplarini_oku(ser, tampon, verbose):
    """Mega cevaplarini bosalt; yalniz gerekli olanlari terminale yaz."""
    veri = ser.read(ser.in_waiting or 1)
    if not veri:
        return tampon

    tampon += veri
    while b"\n" in tampon:
        satir, tampon = tampon.split(b"\n", 1)
        metin = satir.decode("utf-8", errors="replace").strip()
        if metin and (verbose or metin.startswith(("ERR", "WARN", "BOOT"))):
            print(f"MEGA > {metin}")
    return tampon


def foto_cek(rpicam_bin, photo_dir, width, height, timeout_ms):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    dosya_adi = f"foto_{timestamp}.jpg"
    hedef_yol = os.path.join(photo_dir, dosya_adi)

    if rpicam_bin:
        cmd = [
            rpicam_bin,
            "-o", hedef_yol,
            "-t", str(timeout_ms),
            "--width", str(width),
            "--height", str(height),
            "-n",
        ]
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"📸 Fotoğraf çekiliyor -> {hedef_yol}")
        except Exception as e:
            print(f"HATA: Fotoğraf çekme komutu çalıştırılamadı: {e}", file=sys.stderr)
    else:
        print("WARN: Fotoğraf Çekilemedi! Sistemde 'rpicam-still' veya 'libcamera-still' bulunamadı.", file=sys.stderr)


def ana_program():
    args = argumanlari_oku()
    ekran = None
    ser = None

    # Fotograf klasorunu hazirla
    photo_dir = os.path.expanduser(args.photo_dir)
    os.makedirs(photo_dir, exist_ok=True)
    rpicam_bin = shutil.which("rpicam-still") or shutil.which("libcamera-still")

    try:
        ekran, tus_kodlari = x11_ekranini_ac(args.display)

        ser = serial.Serial(
            args.port,
            args.baud,
            timeout=0,
            write_timeout=0.25,
        )

        # Mega port acilinca resetlenebilir; setup() tamamlanana kadar bekle.
        print(f"Mega bekleniyor: {args.port} @ {args.baud} ...")
        time.sleep(2.0)
        ser.reset_input_buffer()
        komut_gonder(ser, "MOTOR:STOP")
        ser.flush()

        komutlar = {
            "w": f"MOTOR:FWD:{args.forward_pwm}",
            "s": f"MOTOR:BACK:{args.back_pwm}",
            "a": f"MOTOR:LEFT:{args.turn_pwm}",
            "d": f"MOTOR:RIGHT:{args.turn_pwm}",
        }

        print(
            "Hazir: VNC'de W/A/S/D ile hareket | P: Fotoğraf Çek | "
            "ESC veya Ctrl+C: cikis"
        )
        print(f"X11 ekrani: {ekran.get_display_name()}")
        print(f"Fotoğraf klasoru: {photo_dir} ({args.photo_width}x{args.photo_height})")
        print("Guvenlik: Kontrol icin bu terminal VNC'de odakta kalmali.")

        kontrol_odagi = x11_odagini_oku(ekran)
        onceki_tuslar = set()
        basili_tuslar = {}
        basma_sirasi = 0
        son_komut = "MOTOR:STOP"
        son_gonderim = time.monotonic()
        tekrar_s = args.repeat_ms / 1000.0
        seri_tamponu = b""
        calisiyor = True

        while calisiyor:
            simdi = time.monotonic()
            bekleme = KLAVYE_YOKLAMA_S
            if son_komut != "MOTOR:STOP":
                tekrar_kalan = tekrar_s - (simdi - son_gonderim)
                bekleme = min(bekleme, max(0.0, tekrar_kalan))

            okunabilir, _, _ = select.select(
                [ser.fileno()], [], [], bekleme
            )
            if okunabilir:
                seri_tamponu = seri_cevaplarini_oku(
                    ser, seri_tamponu, args.verbose
                )

            mevcut_tuslar = x11_basili_tuslari_oku(ekran, tus_kodlari)
            if x11_odagini_oku(ekran) != kontrol_odagi:
                mevcut_tuslar = set()

            if "esc" in mevcut_tuslar and "esc" not in onceki_tuslar:
                calisiyor = False
            if (
                "c" in mevcut_tuslar
                and ("ctrl_sol" in mevcut_tuslar or "ctrl_sag" in mevcut_tuslar)
            ):
                calisiyor = False

            # P tusu tekil tetikleme (Edge-trigger) Fotoğraf cek
            if "p" in mevcut_tuslar and "p" not in onceki_tuslar:
                foto_cek(rpicam_bin, photo_dir, args.photo_width, args.photo_height, args.photo_timeout)

            mevcut_hareket_tuslari = set(komutlar) & mevcut_tuslar
            onceki_hareket_tuslari = set(komutlar) & onceki_tuslar

            for tus in mevcut_hareket_tuslari - onceki_hareket_tuslari:
                basma_sirasi += 1
                basili_tuslar[tus] = basma_sirasi
            for tus in onceki_hareket_tuslari - mevcut_hareket_tuslari:
                basili_tuslar.pop(tus, None)

            onceki_tuslar = mevcut_tuslar

            if basili_tuslar and calisiyor:
                aktif_tus = max(basili_tuslar, key=basili_tuslar.get)
                istenen_komut = komutlar[aktif_tus]
            else:
                aktif_tus = None
                istenen_komut = "MOTOR:STOP"

            simdi = time.monotonic()
            komut_degisti = istenen_komut != son_komut
            tekrar_zamani = (
                istenen_komut != "MOTOR:STOP"
                and simdi - son_gonderim >= tekrar_s
            )

            if komut_degisti or tekrar_zamani:
                komut_gonder(ser, istenen_komut)
                son_gonderim = simdi

                if komut_degisti:
                    durum = TUS_ADLARI.get(aktif_tus, "DURDU")
                    print(f"DURUM: {durum}")
                    son_komut = istenen_komut

        return 0

    except KeyboardInterrupt:
        print("\nCtrl+C algilandi.")
        return 0
    except (RuntimeError, PermissionError, serial.SerialException, OSError) as hata:
        print(f"HATA: {hata}", file=sys.stderr)
        return 1
    finally:
        if ser is not None:
            try:
                if ser.is_open:
                    komut_gonder(ser, "MOTOR:STOP")
                    ser.flush()
                    time.sleep(0.05)
                    ser.close()
            except (serial.SerialException, OSError):
                pass

        if ekran is not None:
            try:
                ekran.close()
            except Exception:
                pass

        print("Rover durduruldu.")


if __name__ == "__main__":
    raise SystemExit(ana_program())

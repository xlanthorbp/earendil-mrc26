#!/usr/bin/env python3
"""VNC/X11 uzerinden arayuzsiz ve dusuk yuklu WASD rover kontrolu ve Fotoğraf Çekme (ROS 2 Node)."""

import os
import shutil
import subprocess
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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


class ManualTeleopNode(Node):
    def __init__(self):
        super().__init__('manual_teleop')

        self.declare_parameter('forward_pwm', 80)
        self.declare_parameter('back_pwm', 80)
        self.declare_parameter('turn_pwm', 60)
        self.declare_parameter('repeat_ms', 100)
        self.declare_parameter('motor_cmd_topic', '/motor/command')
        self.declare_parameter('display', '')
        self.declare_parameter('photo_dir', os.path.expanduser('~/rover_photos'))
        self.declare_parameter('photo_width', 1920)
        self.declare_parameter('photo_height', 1080)
        self.declare_parameter('photo_timeout_ms', 500)

        self.forward_pwm = self.get_parameter('forward_pwm').value
        self.back_pwm = self.get_parameter('back_pwm').value
        self.turn_pwm = self.get_parameter('turn_pwm').value
        self.repeat_ms = self.get_parameter('repeat_ms').value
        self.cmd_topic = self.get_parameter('motor_cmd_topic').value
        self.display_name = self.get_parameter('display').value or None
        self.photo_dir = os.path.expanduser(self.get_parameter('photo_dir').value)
        self.photo_width = self.get_parameter('photo_width').value
        self.photo_height = self.get_parameter('photo_height').value
        self.photo_timeout_ms = self.get_parameter('photo_timeout_ms').value

        # Fotoğraf kaydedilecek klasörü hazırla
        os.makedirs(self.photo_dir, exist_ok=True)
        self.rpicam_bin = shutil.which('rpicam-still') or shutil.which('libcamera-still')

        self.cmd_pub = self.create_publisher(String, self.cmd_topic, 10)

        self.komutlar = {
            "w": f"MOTOR:FWD:{self.forward_pwm}",
            "s": f"MOTOR:BACK:{self.back_pwm}",
            "a": f"MOTOR:LEFT:{self.turn_pwm}",
            "d": f"MOTOR:RIGHT:{self.turn_pwm}",
        }

        self.ekran = None
        self.tus_kodlari = None
        self._init_x11()

        self.kontrol_odagi = x11_odagini_oku(self.ekran)
        self.onceki_tuslar = set()
        self.basili_tuslar = {}
        self.basma_sirasi = 0
        self.son_komut = "MOTOR:STOP"
        self.son_gonderim = time.monotonic()
        self.tekrar_s = self.repeat_ms / 1000.0

        # ROS timer (20ms loop)
        self.timer = self.create_timer(KLAVYE_YOKLAMA_S, self.loop_callback)

        self.get_logger().info(
            f"Manual Teleop Düğümü Başlatıldı.\n"
            f"  - Konu (Topic): {self.cmd_topic}\n"
            f"  - Fotoğraf Klasörü: {self.photo_dir}\n"
            f"  - Fotoğraf Çözünürlüğü: {self.photo_width}x{self.photo_height}\n"
            f"  - X11 Ekranı: {self.ekran.get_display_name()}\n"
            f"  - Hazır: W/A/S/D ile hareket | P: Fotoğraf Çek | ESC veya Ctrl+C: Çıkış"
        )

    def _init_x11(self):
        self.ekran, self.tus_kodlari = x11_ekranini_ac(self.display_name)

    def gonder(self, komut: str):
        msg = String()
        msg.data = komut
        self.cmd_pub.publish(msg)

    def foto_cek(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        dosya_adi = f"foto_{timestamp}.jpg"
        hedef_yol = os.path.join(self.photo_dir, dosya_adi)

        if self.rpicam_bin:
            cmd = [
                self.rpicam_bin,
                "-o", hedef_yol,
                "-t", str(self.photo_timeout_ms),
                "--width", str(self.photo_width),
                "--height", str(self.photo_height),
                "-n",
            ]
            try:
                subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.get_logger().info(f"📸 Fotoğraf çekiliyor -> {hedef_yol}")
            except Exception as e:
                self.get_logger().error(f"Fotoğraf çekme komutu çalıştırılamadı: {e}")
        else:
            self.get_logger().warn(
                "📸 Fotoğraf Çekilemedi: Sistemde 'rpicam-still' veya 'libcamera-still' komutu bulunamadı."
            )

    def loop_callback(self):
        mevcut_tuslar = set()

        try:
            mevcut_tuslar = x11_basili_tuslari_oku(self.ekran, self.tus_kodlari)
            if x11_odagini_oku(self.ekran) != self.kontrol_odagi:
                mevcut_tuslar = set()
        except RuntimeError as e:
            self.get_logger().error(f"X11 Hatası: {e}")
            self.stop_and_exit()
            return

        if "esc" in mevcut_tuslar and "esc" not in self.onceki_tuslar:
            self.stop_and_exit()
            return
        if (
            "c" in mevcut_tuslar
            and ("ctrl_sol" in mevcut_tuslar or "ctrl_sag" in mevcut_tuslar)
        ):
            self.stop_and_exit()
            return

        # P tuşuna ilk basıldığı an (Edge-trigger) Fotoğraf çek
        if "p" in mevcut_tuslar and "p" not in self.onceki_tuslar:
            self.foto_cek()

        mevcut_hareket_tuslari = set(self.komutlar) & mevcut_tuslar
        onceki_hareket_tuslari = set(self.komutlar) & self.onceki_tuslar

        for tus in mevcut_hareket_tuslari - onceki_hareket_tuslari:
            self.basma_sirasi += 1
            self.basili_tuslar[tus] = self.basma_sirasi
        for tus in onceki_hareket_tuslari - mevcut_hareket_tuslari:
            self.basili_tuslar.pop(tus, None)

        self.onceki_tuslar = mevcut_tuslar

        if self.basili_tuslar:
            aktif_tus = max(self.basili_tuslar, key=self.basili_tuslar.get)
            istenen_komut = self.komutlar[aktif_tus]
        else:
            aktif_tus = None
            istenen_komut = "MOTOR:STOP"

        simdi = time.monotonic()
        komut_degisti = istenen_komut != self.son_komut
        tekrar_zamani = (
            istenen_komut != "MOTOR:STOP"
            and simdi - self.son_gonderim >= self.tekrar_s
        )

        if komut_degisti or tekrar_zamani:
            self.gonder(istenen_komut)
            self.son_gonderim = simdi

            if komut_degisti:
                durum = TUS_ADLARI.get(aktif_tus, "DURDU")
                self.get_logger().info(f"DURUM: {durum} ({istenen_komut})")
                self.son_komut = istenen_komut

    def stop_and_exit(self):
        self.gonder("MOTOR:STOP")
        if self.ekran:
            try:
                self.ekran.close()
            except Exception:
                pass
        self.get_logger().info("Rover durduruldu. Çıkış yapılıyor...")
        raise SystemExit(0)

    def destroy_node(self):
        self.gonder("MOTOR:STOP")
        if self.ekran:
            try:
                self.ekran.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ManualTeleopNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    except Exception as e:
        print(f"HATA: {e}", file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()